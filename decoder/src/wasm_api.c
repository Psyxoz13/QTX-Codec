static inline i32 base_target(i32 target) { return target >= TARGET_ETC2_RGB_SPLIT ? target - TARGET_ETC2_RGB_SPLIT + TARGET_ETC2_RGB : target; }

static u64 output_bytes(const Header* header, i32 target)
{
    return level_bytes(header->width, header->height, target);
}

static u64 lower_levels_bytes(i32 width, i32 height, i32 target)
{
    u64 total = 0;
    while (width > 1 || height > 1)
    {
        width = max32(1, width >> 1);
        height = max32(1, height >> 1);
        total += level_bytes(width, height, target);
    }
    return total;
}

static i32 wasm_length(u64 length) { return length <= 0x7FFFFFFF ? (i32)length : -1; }

extern u8 __heap_base;
enum { WASM_PAGE_SIZE = 65536 };
static size_t heap_top = (size_t)&__heap_base;
static u8 header_buffer[64];

__attribute__((export_name("allocate")))
void* allocate(size_t size)
{
    if (size > 0x7FFFFFFF)
        return 0;
    size_t start = (heap_top + 15) & ~(size_t)15;
    size_t end = start + size;
    size_t available = __builtin_wasm_memory_size(0) * WASM_PAGE_SIZE;
    if (end < start || end > 0x7FFFFFFF)
        return 0;
    if (end > available && __builtin_wasm_memory_grow(0, (end - available + WASM_PAGE_SIZE - 1) / WASM_PAGE_SIZE) == (size_t)-1)
        return 0;
    heap_top = end;
    return (void*)start;
}

__attribute__((export_name("release")))
void release(void)
{
    heap_top = (size_t)&__heap_base;
}

__attribute__((export_name("qtx_header_buffer")))
u8* qtx_header_buffer(void)
{
    return header_buffer;
}

__attribute__((export_name("qtx_output_length")))
i32 qtx_output_length(i32 target)
{
    Header header;
    if (target < 0 || target >= API_TARGET_COUNT || !read_header(header_buffer, (i32)sizeof(header_buffer), &header))
        return -1;
    return wasm_length(output_bytes(&header, base_target(target)));
}

__attribute__((export_name("qtx_prepare")))
i32 qtx_prepare(i32 target)
{
    Header header;
    if (!initialized)
        initialize();
    heap_top = (size_t)&__heap_base;
    return target >= 0 && target < API_TARGET_COUNT && read_header(header_buffer, (i32)sizeof(header_buffer), &header) && wasm_length(output_bytes(&header, base_target(target))) >= 0;
}

static Decoding wasm_decoding;
static u8* wasm_scratch;

__attribute__((export_name("qtx_decode_begin")))
i32 qtx_decode_begin(const u8* file, i32 file_length, i32 inherit)
{
    Header header;
    if (!initialized || !read_header(file, file_length, &header))
        return 0;
    wasm_scratch = allocate(scratch_bytes(&header));
    if (!wasm_scratch || begin_decoding(file, file_length, wasm_scratch, &wasm_decoding, inherit) != 1)
        return 0;
    return wasm_decoding.stripe_count;
}

__attribute__((noinline)) static i32 decode_wasm_stripes(i32 target, u8* out, i32 first_stripe, i32 stripe_end)
{
    i32 alpha = wasm_decoding.header.has_alpha;
    if (target == TARGET_BC1 && !alpha)
        return decode_stripes(&wasm_decoding, TARGET_BC1, 0, out, wasm_scratch, first_stripe, stripe_end);
    if (target == TARGET_BC3 && alpha)
        return decode_stripes(&wasm_decoding, TARGET_BC3, 1, out, wasm_scratch, first_stripe, stripe_end);
    if (target == TARGET_ASTC_4X4)
        return alpha ? decode_stripes(&wasm_decoding, TARGET_ASTC_4X4, 1, out, wasm_scratch, first_stripe, stripe_end) : decode_stripes(&wasm_decoding, TARGET_ASTC_4X4, 0, out, wasm_scratch, first_stripe, stripe_end);
    if (alpha)
        return decode_stripes(&wasm_decoding, target, 1, out, wasm_scratch, first_stripe, stripe_end);
    return decode_stripes(&wasm_decoding, target, 0, out, wasm_scratch, first_stripe, stripe_end);
}

__attribute__((export_name("qtx_decode_stripes")))
i32 qtx_decode_stripes(i32 target, u8* out, i32 first_stripe, i32 stripe_end)
{
    if (!wasm_scratch || target < 0 || target >= API_TARGET_COUNT || wasm_length(output_bytes(&wasm_decoding.header, base_target(target))) < 0)
        return 0;
    i32 block = large_block_of(target);
    if (block)
    {
        u32* rows = allocate(large_rows_bytes(&wasm_decoding.header, block));
        return rows && decode_large_stripes(&wasm_decoding, target, out, wasm_scratch, rows, first_stripe, stripe_end) == 1;
    }
    wasm_decoding.header.etc_split = target >= TARGET_ETC2_RGB_SPLIT;
    return decode_wasm_stripes(base_target(target), out, first_stripe, stripe_end) == 1;
}

__attribute__((export_name("qtx_mips_output_length")))
i32 qtx_mips_output_length(i32 target)
{
    Header header;
    if (target < 0 || target >= API_TARGET_COUNT || !read_header(header_buffer, (i32)sizeof(header_buffer), &header))
        return -1;
    return wasm_length(output_bytes(&header, base_target(target)) + lower_levels_bytes(header.width, header.height, base_target(target)));
}

__attribute__((export_name("qtx_mips_work_length")))
i32 qtx_mips_work_length(void)
{
    Header header;
    if (!read_header(header_buffer, (i32)sizeof(header_buffer), &header))
        return -1;
    return wasm_length(mip_work_bytes(header.width, header.height));
}

__attribute__((export_name("qtx_set_mip_side")))
void qtx_set_mip_side(u32* side)
{
    wasm_decoding.header.mip_side = side;
    wasm_decoding.header.side_width = max32(1, wasm_decoding.header.width >> 1);
    wasm_decoding.header.side_height = max32(1, wasm_decoding.header.height >> 1);
}

__attribute__((export_name("qtx_build_mips")))
i32 qtx_build_mips(i32 target, u8* out)
{
    if (!wasm_decoding.header.mip_side || target < 0 || target >= API_TARGET_COUNT)
        return 0;
    build_lower_levels(wasm_decoding.header.mip_side, wasm_decoding.header.width, wasm_decoding.header.height, base_target(target), wasm_decoding.header.has_alpha, out);
    wasm_decoding.header.mip_side = 0;
    return 1;
}

__attribute__((export_name("qtx_decode_file")))
i32 qtx_decode_file(const u8* file, i32 file_length, i32 target, u8* out)
{
    i32 stripes = qtx_decode_begin(file, file_length, 0);
    return stripes > 0 && target >= 0 && target < API_TARGET_COUNT && qtx_decode_stripes(target, out, 0, stripes);
}
