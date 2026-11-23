# QTX design notes

A QTX texture is a grid of 4x4 blocks. Each block is stored as a primitive that every GPU block format can express directly. At load time the decoder reads the primitives once and writes them out as blocks of the requested format. There's no endpoint, partition or mode search at load time: the encoder makes those choices up front, for all targets at once.

The stream is laid out so that a CPU can decode it with few unpredictable branches and several independent dependency chains at once, and so that one texture can be transcoded on all cores. Transcoding speed is the main goal of the format, and the encoder spends its bits where they matter for the formats GPUs use most: BC1/BC3 and BC7 on desktop, ASTC 4x4 on mobile.

## Primitives

### Line

A line is a segment in RGBA space, stored as a center and an extent, plus a 2-bit tone for every pixel. Tone 0 is the low end, tone 3 the high end, and 1 and 2 are the points at one and two thirds. All GPU block formats have this shape, two endpoints with per-pixel weights. BC1/BC3 and BC7 take it more or less as is, ASTC 4x4 takes it as a single-plane block, and ETC2 gets it through the base color and modifier table or the H mode.

There are three ways to store the line itself:

- Free line: the center and extent are a prediction plus quantized residuals. The quantization step depends on an activity class, one of four, picked from how much the block above differs from its neighbours.
- Table line: an RGB555 center with one of 8 fixed extents equal to the ETC modifier tables, so it comes out exactly the same in ETC2.
- Pair: in images with flat colors a line can take its two ends from the exact colors of the lines and solid blocks to the left, above, above left and above right, or transparent black. It costs one symbol for an ordered pair.

The 16 tones form a 32-bit pattern, stored in one of three ways:

- Codebook: one of 8192 patterns, or its inverse. The patterns were trained on a large set of images and cover edges, ramps and the other shapes a 4x4 block usually has. The code is sent as a bucket inside the block's head symbol plus a few raw bits, so the most common patterns cost no symbol of their own.
- Literal: four row residuals, each row predicted from the one above, and the first row from the bottom row of the block above.
- Binary: a 16-bit mask for blocks with only the two end tones, used in images with alpha or flat colors.

A dual block gives alpha its own pattern with 8 levels between the alpha ends instead of 4. It is stored as a coarse 2-bit pattern, whose rows are differences from the color rows, plus one refinement bit per pixel, so the coarse pattern is also the best 4-level approximation. BC3 writes all 8 levels to its BC4 alpha block, and ETC2 writes them to its EAC alpha block with the base, multiplier and modifier table that fit the 8 levels best, taken from tables computed ahead of time for every alpha range. BC7 writes the block in mode 4 with 3-bit alpha indices when that is closer than mode 5 with the coarse pattern, and ASTC uses the coarse pattern.

### Smooth block

A smooth block stores only its center color. Its colors come from the centers of the eight blocks around it: each corner of the block is a weighted average of its own center and the three neighbours that share that corner. Transparent neighbours count as the block itself, and so do neighbours outside the block's stripe. The decoder takes the two corners that are farthest apart as line endpoints and projects the bilinear surface through the corners onto them for tones. A smooth gradient costs one center per block and has no seams at block edges. For ETC2 a smooth block becomes a planar block directly. For RGBA8 and ASTC the tones are dithered with a 4x4 Bayer matrix, which keeps very shallow gradients from turning into steps.

### Solid, transparent and copies

A solid block is a single color. A transparent block is zero in all channels.

A copy block repeats the block on its left or the block above exactly. The decoder copies the already transcoded GPU block, so flat or repeated areas cost one cheap symbol and a memory copy per block. Smooth blocks are never copied, because their colors depend on their neighbours.

A run repeats a transparent block or a copy for at least four more blocks. Its length follows as a bucket symbol plus raw bits.

## Prediction

Centers and extents are predicted from blocks that are already decoded, in one of two modes. Images with alpha always use the left mode. For other images the encoder warms up its cost models on the middle stripe in both modes and keeps the mode whose blocks cost less in distortion plus lambda times bits.

- Up mode: the prediction is a 1-2-1 average of the three blocks above. It doesn't depend on anything in the same row, so a whole row is reconstructed in one pass without a chain from block to block.
- Left mode: the prediction is the median of the left, upper and upper left blocks (the MED predictor), and the opcode context also includes the class of the block on the left. Sprites and interface art have long horizontal runs of the same color, and photos with smooth areas gain from the copies and pairs this mode allows; noisy material textures usually stay in the up mode.

Blocks that carry no value of their own still carry one for their neighbours. A transparent block takes the color and extent of the block above it, or of the block on its left in the first row of a stripe, so the edge of a sprite is predicted from the sprite and not from black. Solid and smooth blocks keep the predicted extent instead of zero. This costs nothing to store and keeps predictions close across transparent holes and flat areas.

## Stream layout

### Head symbols

Every block starts with a head symbol: transparent, smooth, solid, a codebook line with its code bucket, a table line with its code bucket, a literal line, a literal table line, a dual line, a pair, a binary line, a binary table line, copy left, copy up or run. The head alphabet has nine contexts. In the up mode they come from the classes of the blocks above and above right, in the left mode from the blocks on the left and above.

### Planes and lanes

The symbols of a stripe are split into four planes: heads, centers, extents and shapes. Each plane is split into eight lanes, each with its own rANS state and its own streams of 16-bit words and raw bits. Block `x` of a row belongs to lane `x mod 8`, so the eight lanes always share the work evenly, whatever the image looks like, and the left neighbour of a block is always already known.

The decoder reads a row plane by plane:

- Heads are read in raster order. Consecutive blocks use different lanes, so the rANS state updates of eight blocks overlap; only the head context waits for the block on the left. This pass also builds, per lane, the lists of blocks that have a center, an extent or a shape.
- Centers, extents and shapes are each read over those lists, one block per lane at a time.

Residual vectors are coded as one joint symbol for the common case where all three decorrelated components are within two steps, and an escape otherwise. Escapes are read at the end of the row, so the main loop reads joint symbols without branching on them. Alpha residuals are read as bucket symbols plus raw bits. Blocks whose center or extent is implied by their kind have no symbol at all.

Activity classes depend only on the row above, and literal row contexts only on the pattern above and on the previous row of the same pattern, so every plane can be decoded before the row is reconstructed.

### Entropy coding

All symbols go through a static rANS coder with 11-bit probabilities and branch-free renormalization. Frequency tables are stored compactly, with each symbol's frequency picked from a small set of classes, and the decoder expands every table into a 2048-entry lookup.

### Stripes

The image is cut into stripes of 16 block rows by default. The height is stored in the header, so the decoder reads any stripe height. Stripes don't depend on each other in any way: predictions, contexts and smooth blocks stay inside their stripe. Every stripe can be found through a small directory and decoded on its own, in any order or in parallel. `qtx_transcode_units` transcodes any range of stripes into the shared output buffer, so a single texture can be spread over all cores, each with its own scratch buffer.

## Mip levels

A mip container (`QTXM`) stores a level count, the size of each level and then the levels as ordinary QTX streams. Levels after the first reuse the frequency tables of level 0, which are built from statistics of the whole chain. For `qtx_transcode_units` the stripes of all stored levels are numbered one after the other.

A file with a single level can also produce its mips while it's being transcoded. The decoder writes the half-size level as a side product of level 0, using the primitives directly, and every further level is a 2x2 average of the previous one. Each block of these levels gets a quick line fit and goes through the normal transcoder for the target format.

## Encoder

The encoder does rate-distortion optimization block by block. For every candidate it computes distortion plus penalties plus lambda times bits. In images without alpha, distortion weights the channels by how much they contribute to brightness, red 5, green 10 and blue 2, because brightness carries most of the visible detail; images with alpha, which are mostly sprites and interface art with saturated colors, weight the color channels equally. Alpha gets a weight of its own. A structure term compares the local contrast and structure of the block with the source; for smooth blocks it counts twice, because their interpolated surface loses the texture inside the block that SSIM measures. The other penalties cover alpha errors, lines that ETC2 can't reproduce well, and edges. The bit costs come from adaptive models that are warmed up on the image first. Copies compete with the other candidates like any other choice. For pairs the encoder rates every pair of neighbour colors by its ideal error and searches a pattern only for the closest pair, in the order whose pair symbol costs less.

For a line pattern the encoder searches the whole codebook with per-row error tables, and compares it with the literal and binary patterns. The codebook is grouped by the first row of a pattern, and a group is skipped when its first-row error plus the smallest errors of the other three rows already can't beat the best pattern found; every encoder build visits the groups in the same order, so they all choose the same pattern. A smooth block is estimated from the already chosen blocks and the targets of the blocks still to come; it is only chosen where the final neighbours can't change the alpha of an opaque block. Quality maps to lambda as 60 * 10^((50 - quality) / 30) times the average color channel weight, so a quality setting means about the same for both kinds of images, and the quantization steps scale with lambda. In the up mode the center steps start coarser than in the left mode: at the same file size, the bits that coarser centers save buy more PSNR and SSIM in extents and patterns on photos and material textures, while sprites and flat colors need exact centers.

## Decoder

The decoder is one C file plus the transcoders. It doesn't use the C library and builds for Windows, Linux, macOS, Android, iOS and WebAssembly (with and without SIMD).

For every block row it decodes the four planes, reconstructs the row and writes target blocks straight into the output. Line blocks and smooth blocks of opaque images go through dedicated loops for BC1, BC7, ASTC and ETC2, and smooth blocks are processed four at a time with SIMD. The ETC2 transcoder picks the modifier table directly from the line extent and only tries the H mode for lines with strong color shifts.

In the browser the decoder runs in a pool of workers. The wasm module is compiled once and shared by all of them, and output buffers are sent back to the workers and reused, so a texture doesn't have to wait for fresh memory to be zeroed.

## ASTC 6x6 and 8x8

Blocks of 6x6 and 8x8 texels don't line up with the 4x4 primitives, so these targets work differently: the decoder writes a stripe as RGBA8 rows into scratch memory and a small ASTC encoder fits the blocks from those rows. Unlike the other targets this is an encoding step at load time, but a narrow one, with a fixed set of modes and no search beyond it.

- Blocks with a single color or with zero alpha everywhere become void-extent blocks.
- A block gets one line: the principal axis of its colors, found by power iteration, and the position of every texel along it as its ideal weight. The ideal weights are reduced to the mode's weight grid with the least-squares solution for the ASTC weight infill, which is separable, so it is two small one-dimensional pseudo-inverses. The grid weights are quantized to the mode's levels, spread back to the texels, and the endpoints are the least-squares fit to those weights, quantized to the endpoint levels the mode leaves room for.
- Each block size has eight modes for opaque blocks and eight for blocks with alpha, chosen from all single-partition modes by measurement on a training set: greedily, adding each time the mode that lowers the total error most. For an opaque block every mode gets an estimate from the 2D DCT of the ideal weights, the energy at frequencies the grid can't hold plus the quantization noise of its levels, and only the three best are fitted. Two-partition modes added 0.07 dB on 8x8 and were left out.
- In blocks with alpha the color error is weighted by alpha squared, the way it shows after blending, and some of the modes, three of eight for 8x8 and five for 6x6, have a second weight plane for alpha, so sprite edges stay sharp while the color follows its own line. All eight modes are fitted, since these blocks are few.

A default stripe of 16 block rows, 64 texels, is a whole number of 8x8 blocks but not of 6x6 ones, so for 6x6 a work unit covers three stripes and the rows left over at the end of a stripe are carried into the next one. The fitting uses floating point with contraction into fused multiply-add turned off, so every build, x86, ARM and WebAssembly, writes the same blocks.

Byte 21 of the first level's header is left for the application to record the ASTC block size it found best for the image: 4, 6 or 8, and 0 when nothing was measured. The decoder doesn't read it. UniQTX writes it at import after measuring all three sizes against the source, and picks that size on devices with ASTC.
