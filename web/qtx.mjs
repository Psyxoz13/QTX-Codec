export const QtxFormat = Object.freeze({
  RGBA8: 0,
  BC1_RGB: 1,
  BC3_RGBA: 2,
  ETC2_RGB: 3,
  ETC2_RGBA: 4,
  ASTC_4x4_RGBA: 5,
  BC7_RGBA: 6,
  ASTC_6x6_RGBA: 7,
  ASTC_8x8_RGBA: 8,
});

export const QtxTranscodeFlags = Object.freeze({
  DEFAULT: 0,
  BUILD_MIPS: 1,
  HIGH_QUALITY: 2,
});

const HEADER_BYTES = 24;
const MIP_HEADER_BYTES = 8;
const MAX_LEVELS = 16;
const FORMAT_VERSION = 1;
const BLOCK_BYTES = [64, 8, 16, 8, 16, 16, 16, 16, 16];
const BLOCK_SIDES = [4, 4, 4, 4, 4, 4, 4, 6, 8];
const WASM_HIGH_QUALITY_TARGETS = { [QtxFormat.ETC2_RGB]: 9, [QtxFormat.ETC2_RGBA]: 10 };
const SIMD_PROBE = new Uint8Array([0, 97, 115, 109, 1, 0, 0, 0, 1, 5, 1, 96, 0, 1, 123, 3, 2, 1, 0, 10, 10, 1, 8, 0, 65, 0, 253, 15, 253, 98, 11]);

export function supportsSimd() {
  return WebAssembly.validate(SIMD_PROBE);
}

export async function createTranscoder(wasm) {
  const result = await WebAssembly.instantiate(wasm, {});
  const instance = result instanceof WebAssembly.Instance ? result : result.instance;
  return new QtxTranscoder(instance.exports);
}

function hasMagic(bytes, offset, last) {
  return bytes.byteLength - offset >= 4 && bytes[offset] === 81 && bytes[offset + 1] === 84 && bytes[offset + 2] === 88 && bytes[offset + 3] === last;
}

function storedLevels(bytes) {
  if (!hasMagic(bytes, 0, 77))
    return [{ offset: 0, length: bytes.byteLength }];
  const count = bytes.byteLength >= MIP_HEADER_BYTES ? bytes[4] : 0;
  let offset = MIP_HEADER_BYTES + count * 4;
  if (count < 1 || count > MAX_LEVELS || bytes.byteLength < offset)
    throw new Error('QTX mip container is corrupt');
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const levels = [];
  for (let level = 0; level < count; level++) {
    const length = view.getUint32(MIP_HEADER_BYTES + level * 4, true);
    if (length > bytes.byteLength - offset)
      throw new Error('QTX mip container is truncated');
    levels.push({ offset, length });
    offset += length;
  }
  return levels;
}

function levelHeader(bytes, level) {
  if (level.length < HEADER_BYTES || !hasMagic(bytes, level.offset, 32))
    throw new Error('the data is not a QTX image');
  const at = level.offset;
  if (bytes[at + 8] !== FORMAT_VERSION)
    throw new Error(`QTX format version ${bytes[at + 8]} is not supported by this decoder`);
  const width = bytes[at + 4] | (bytes[at + 5] << 8), height = bytes[at + 6] | (bytes[at + 7] << 8);
  if (!width || !height || !bytes[at + 19])
    throw new Error('QTX header is corrupt');
  return { width, height, hasAlpha: (bytes[at + 9] & 1) !== 0 };
}

function levelSize(width, height, format) {
  const side = BLOCK_SIDES[format];
  return format === QtxFormat.RGBA8 ? width * height * 4 : Math.ceil(width / side) * Math.ceil(height / side) * BLOCK_BYTES[format];
}

function outputLayout(dimensions, format) {
  let offset = 0;
  return dimensions.map(({ width, height }) => {
    const size = levelSize(width, height, format);
    const level = { width, height, offset, size };
    offset += size;
    return level;
  });
}

function mipChain(width, height) {
  const chain = [{ width, height }];
  while (width > 1 || height > 1) {
    width = Math.max(1, width >> 1);
    height = Math.max(1, height >> 1);
    chain.push({ width, height });
  }
  return chain;
}

export class QtxTranscoder {
  #exports;
  #spareOutputs = [];
  #spareOutputLimit = 2;

  constructor(exports) {
    this.#exports = exports;
  }

  getImageInfo(bytes) {
    const levels = storedLevels(bytes);
    const header = levels.map(level => levelHeader(bytes, level))[0];
    return { width: header.width, height: header.height, levelCount: levels.length, hasAlpha: header.hasAlpha, formatVersion: FORMAT_VERSION };
  }

  transcode(bytes, format, flags = QtxTranscodeFlags.DEFAULT) {
    if (!Number.isInteger(format) || format < 0 || format >= BLOCK_BYTES.length)
      throw new Error(`QTX format ${format} is not supported`);
    if (!Number.isInteger(flags) || flags < 0 || flags > (QtxTranscodeFlags.BUILD_MIPS | QtxTranscodeFlags.HIGH_QUALITY))
      throw new Error(`QTX flags ${flags} are not supported`);
    const levels = storedLevels(bytes);
    const headers = levels.map(level => levelHeader(bytes, level));
    const target = flags & QtxTranscodeFlags.HIGH_QUALITY ? WASM_HIGH_QUALITY_TARGETS[format] ?? format : format;
    this.#loadHeader(bytes, levels[0]);
    if (!this.#exports.qtx_prepare(target))
      throw new Error(`QTX header or format ${format} is not supported`);
    if (flags & QtxTranscodeFlags.BUILD_MIPS && levels.length === 1) {
      const layout = outputLayout(mipChain(headers[0].width, headers[0].height), format);
      return { data: this.#transcodeWithBuiltMips(bytes, levels[0], target, layout), levels: layout };
    }
    const layout = outputLayout(headers, format);
    const outputLength = layout.reduce((sum, level) => sum + level.size, 0);
    const output = this.#allocate(outputLength);
    const file = this.#allocate(bytes.byteLength);
    this.#memory().set(bytes, file);
    levels.forEach((level, index) => this.#decodeLevel(file + level.offset, level.length, index > 0, target, output + layout[index].offset));
    return { data: this.#copyOutput(output, outputLength), levels: layout };
  }

  recycleOutput(output) {
    if (this.#spareOutputs.length < this.#spareOutputLimit)
      this.#spareOutputs.push(ArrayBuffer.isView(output) ? output.buffer : output);
  }

  releaseOutputs() {
    this.#spareOutputs.length = 0;
  }

  #transcodeWithBuiltMips(bytes, level, target, layout) {
    const exports = this.#exports;
    const chainLength = exports.qtx_mips_output_length(target);
    const workLength = exports.qtx_mips_work_length();
    if (chainLength < 0 || workLength < 0)
      throw new Error('QTX mip levels cannot be built for this format');
    const chain = this.#allocate(chainLength);
    const file = this.#allocate(bytes.byteLength);
    const work = this.#allocate(workLength);
    this.#memory().set(bytes, file);
    const stripes = exports.qtx_decode_begin(file + level.offset, level.length, 0);
    if (!stripes)
      throw new Error('QTX data is corrupt or truncated');
    exports.qtx_set_mip_side(work);
    if (!exports.qtx_decode_stripes(target, chain, 0, stripes))
      throw new Error('QTX data is corrupt or truncated');
    if (!exports.qtx_build_mips(target, chain + layout[0].size))
      throw new Error('QTX mip levels could not be built');
    return this.#copyOutput(chain, chainLength);
  }

  #decodeLevel(file, length, inheritTables, target, output) {
    const stripes = this.#exports.qtx_decode_begin(file, length, inheritTables ? 1 : 0);
    if (!stripes)
      throw new Error('QTX data is corrupt or truncated');
    if (!this.#exports.qtx_decode_stripes(target, output, 0, stripes))
      throw new Error('QTX data is corrupt or truncated');
  }

  #loadHeader(bytes, level) {
    this.#memory().set(bytes.subarray(level.offset, level.offset + HEADER_BYTES), this.#exports.qtx_header_buffer());
  }

  #allocate(length) {
    if (!Number.isSafeInteger(length) || length < 1 || length > 0x7FFFFFFF)
      throw new Error('QTX buffer exceeds the WebAssembly size limit');
    const at = this.#exports.allocate(length);
    if (!at)
      throw new Error('WebAssembly memory exhausted');
    return at;
  }

  #memory() {
    return new Uint8Array(this.#exports.memory.buffer);
  }

  #copyOutput(at, length) {
    const view = new Uint8Array(this.#exports.memory.buffer, at, length);
    const index = this.#spareOutputs.findIndex(buffer => buffer.byteLength >= length);
    if (index < 0)
      return view.slice();
    const output = new Uint8Array(this.#spareOutputs.splice(index, 1)[0], 0, length);
    output.set(view);
    return output;
  }
}
