#pragma clang fp contract(off)

#ifndef LARGE_FITTED_MODES
#define LARGE_FITTED_MODES 3
#endif
#ifndef LARGE_REFINE
#define LARGE_REFINE 0
#endif

enum { ASTC_RANGES = 21, ASTC_WEIGHT_RANGES = 12, ASTC_MAX_WEIGHTS = 64, ASTC_MAX_SIDE = 8, ASTC_LARGE_MODES = 8, ASTC_MAX_WEIGHT_LEVELS = 32 };
enum { RANGE_BITS_ONLY, RANGE_TRIT, RANGE_QUINT };

static const u16 ASTC_RANGE_LEVELS[ASTC_RANGES] = { 2, 3, 4, 5, 6, 8, 10, 12, 16, 20, 24, 32, 40, 48, 64, 80, 96, 128, 160, 192, 256 };
static const u8 ASTC_RANGE_BITS[ASTC_RANGES] = { 1, 0, 2, 0, 1, 3, 1, 2, 4, 2, 3, 5, 3, 4, 6, 4, 5, 7, 5, 6, 8 };
static const u8 ASTC_RANGE_KIND[ASTC_RANGES] = { 0, 1, 0, 2, 1, 0, 2, 1, 0, 2, 1, 0, 2, 1, 0, 2, 1, 0, 2, 1, 0 };

static u8 ASTC_QUINT_BLOCK[125];

typedef struct
{
    f4 row[ASTC_MAX_SIDE][2];
} LargePlane;

typedef struct
{
    u8 grid_width, grid_height, weight_count, weight_range, endpoint_range, channels, dual, levels;
    u16 block_mode;
    float weight_noise, endpoint_noise;
    f4 decimate_y[ASTC_MAX_SIDE][ASTC_MAX_SIDE];
    f4 decimate_x[ASTC_MAX_SIDE][ASTC_MAX_SIDE];
    f4 infill_x[ASTC_MAX_SIDE][ASTC_MAX_SIDE];
    f4 infill_y[ASTC_MAX_SIDE][ASTC_MAX_SIDE];
    float thresholds[ASTC_MAX_WEIGHT_LEVELS];
    float steps[ASTC_MAX_WEIGHT_LEVELS];
    u8 sorted_index[ASTC_MAX_WEIGHT_LEVELS];
    u8 weight_complement[ASTC_MAX_WEIGHT_LEVELS];
    u8 endpoint_index_of[256];
    u8 endpoint_value[256];
} AstcLargeMode;

typedef struct
{
    i32 block, rgb_count, rgba_count;
    LargePlane mask;
    f4 dct_y[ASTC_MAX_SIDE][ASTC_MAX_SIDE];
    f4 dct_x[ASTC_MAX_SIDE][ASTC_MAX_SIDE];
    AstcLargeMode rgb[ASTC_LARGE_MODES];
    AstcLargeMode rgba[ASTC_LARGE_MODES];
} AstcLargeEncoder;

static AstcLargeEncoder ASTC_6X6_ENCODER, ASTC_8X8_ENCODER;

static i32 ise_bit_count(i32 count, i32 range)
{
    i32 kind = ASTC_RANGE_KIND[range];
    return count * ASTC_RANGE_BITS[range] + (kind == RANGE_TRIT ? (8 * count + 4) / 5 : kind == RANGE_QUINT ? (7 * count + 2) / 3 : 0);
}

static void decode_quint_block(u32 code, i32 quints[3])
{
    u32 bit0 = code & 1, bit3 = (code >> 3) & 1, bit4 = (code >> 4) & 1;
    if (((code >> 1) & 3) == 3 && ((code >> 5) & 3) == 0)
    {
        quints[0] = 4;
        quints[1] = 4;
        quints[2] = (i32)((bit0 << 2) | ((bit4 & (1 - bit0)) << 1) | (bit3 & (1 - bit0)));
        return;
    }
    u32 c;
    if (((code >> 1) & 3) == 3)
    {
        quints[2] = 4;
        c = (((code >> 3) & 3) << 3) | ((~(code >> 5) & 3) << 1) | bit0;
    }
    else
    {
        quints[2] = (i32)((code >> 5) & 3);
        c = code & 31;
    }
    if ((c & 7) == 5)
    {
        quints[0] = (i32)(c >> 3);
        quints[1] = 4;
    }
    else
    {
        quints[0] = (i32)(c & 7);
        quints[1] = (i32)(c >> 3);
    }
}

static i32 ise_write(u64 stream[2], i32 position, const u8* values, i32 count, i32 range)
{
    static const u8 TRIT_SPLITS[5] = { 2, 2, 1, 2, 1 }, QUINT_SPLITS[3] = { 3, 2, 2 };
    i32 bits = ASTC_RANGE_BITS[range], kind = ASTC_RANGE_KIND[range];
    u32 low_mask = (1u << bits) - 1;
    if (kind == RANGE_BITS_ONLY)
    {
        for (i32 i = 0; i < count; i++)
            put_bits(&stream[0], &stream[1], &position, values[i], bits);
        return position;
    }
    i32 group = kind == RANGE_TRIT ? 5 : 3;
    const u8* splits = kind == RANGE_TRIT ? TRIT_SPLITS : QUINT_SPLITS;
    for (i32 start = 0; start < count; start += group)
    {
        i32 high[5] = { 0, 0, 0, 0, 0 }, length = min32(group, count - start);
        for (i32 k = 0; k < length; k++)
            high[k] = values[start + k] >> bits;
        u32 code = kind == RANGE_TRIT ? ASTC_TRIT_BLOCK[high[0] + 3 * high[1] + 9 * high[2] + 27 * high[3] + 81 * high[4]] : ASTC_QUINT_BLOCK[high[0] + 5 * high[1] + 25 * high[2]];
        i32 used = 0;
        for (i32 k = 0; k < length; k++)
        {
            put_bits(&stream[0], &stream[1], &position, values[start + k] & low_mask, bits);
            put_bits(&stream[0], &stream[1], &position, (code >> used) & ((1u << splits[k]) - 1), splits[k]);
            used += splits[k];
        }
    }
    return position;
}

static i32 replicate_bits(i32 value, i32 bits, i32 width)
{
    i32 out = 0, total = 0;
    while (total < width)
    {
        out = (out << bits) | value;
        total += bits;
    }
    return out >> (total - width);
}

static i32 endpoint_unquantized(i32 range, i32 value)
{
    i32 bits = ASTC_RANGE_BITS[range], kind = ASTC_RANGE_KIND[range];
    if (kind == RANGE_BITS_ONLY)
        return replicate_bits(value, bits, 8);
    i32 d = value >> bits, m = value & ((1 << bits) - 1);
    i32 a = (m & 1) ? 0x1FF : 0;
    i32 b = (m >> 1) & 1, c = (m >> 2) & 1, e = (m >> 3) & 1, f = (m >> 4) & 1, g = (m >> 5) & 1;
    i32 base = 0, scale;
    if (kind == RANGE_TRIT)
    {
        static const i32 SCALES[7] = { 0, 204, 93, 44, 22, 11, 5 };
        scale = SCALES[bits];
        if (bits == 2)
            base = (b << 8) | (b << 4) | (b << 2) | (b << 1);
        else if (bits == 3)
            base = (c << 8) | (b << 7) | (c << 3) | (b << 2) | (c << 1) | b;
        else if (bits == 4)
            base = (e << 8) | (c << 7) | (b << 6) | (e << 2) | (c << 1) | b;
        else if (bits == 5)
            base = (f << 8) | (e << 7) | (c << 6) | (b << 5) | (f << 1) | e;
        else if (bits == 6)
            base = (g << 8) | (f << 7) | (e << 6) | (c << 5) | (b << 4) | g;
    }
    else
    {
        static const i32 SCALES[6] = { 0, 113, 54, 26, 13, 6 };
        scale = SCALES[bits];
        if (bits == 2)
            base = (b << 8) | (b << 3) | (b << 2);
        else if (bits == 3)
            base = (c << 8) | (b << 7) | (c << 2) | (b << 1) | c;
        else if (bits == 4)
            base = (e << 8) | (c << 7) | (b << 6) | (e << 1) | c;
        else if (bits == 5)
            base = (f << 8) | (e << 7) | (c << 6) | (b << 5) | f;
    }
    i32 t = (d * scale + base) ^ a;
    return (a & 0x80) | (t >> 2);
}

static i32 weight_unquantized(i32 range, i32 value)
{
    i32 bits = ASTC_RANGE_BITS[range], kind = ASTC_RANGE_KIND[range], t;
    if (kind == RANGE_BITS_ONLY)
        t = replicate_bits(value, bits, 6);
    else if (bits == 0)
        t = kind == RANGE_TRIT ? (i32[]){ 0, 32, 63 }[value] : (i32[]){ 0, 16, 32, 47, 63 }[value];
    else
    {
        i32 d = value >> bits, m = value & ((1 << bits) - 1);
        i32 a = (m & 1) ? 0x7F : 0, b = (m >> 1) & 1, c = (m >> 2) & 1, base = 0, scale;
        if (kind == RANGE_TRIT)
        {
            scale = bits == 1 ? 50 : bits == 2 ? 23 : 11;
            base = bits == 2 ? (b << 6) | (b << 2) | b : bits == 3 ? (c << 6) | (b << 5) | (c << 1) | b : 0;
        }
        else
        {
            scale = bits == 1 ? 28 : 13;
            base = bits == 2 ? (b << 6) | (b << 1) : 0;
        }
        t = (d * scale + base) ^ a;
        t = (a & 0x20) | (t >> 2);
    }
    return t + (t > 32);
}

static i32 decode_block_mode(i32 mode, i32* width, i32* height, i32* dual, i32* range)
{
    i32 base = (mode >> 4) & 1, high = (mode >> 9) & 1, plane = (mode >> 10) & 1, a = (mode >> 5) & 3, b;
    if (mode & 3)
    {
        base |= (mode & 3) << 1;
        b = (mode >> 7) & 3;
        switch ((mode >> 2) & 3)
        {
            case 0: *width = b + 4; *height = a + 2; break;
            case 1: *width = b + 8; *height = a + 2; break;
            case 2: *width = a + 2; *height = b + 8; break;
            default:
                b &= 1;
                if (mode & 0x100)
                {
                    *width = b + 2;
                    *height = a + 2;
                }
                else
                {
                    *width = a + 2;
                    *height = b + 6;
                }
        }
    }
    else
    {
        base |= ((mode >> 2) & 3) << 1;
        if (((mode >> 2) & 3) == 0)
            return 0;
        b = (mode >> 9) & 3;
        switch ((mode >> 7) & 3)
        {
            case 0: *width = 12; *height = a + 2; break;
            case 1: *width = a + 2; *height = 12; break;
            case 2: *width = a + 6; *height = b + 6; plane = 0; high = 0; break;
            default:
                if (((mode >> 5) & 3) >= 2)
                    return 0;
                *width = ((mode >> 5) & 3) == 0 ? 6 : 10;
                *height = ((mode >> 5) & 3) == 0 ? 10 : 6;
        }
    }
    *range = base - 2 + 6 * high;
    *dual = plane;
    if (*range < 0 || *range >= ASTC_WEIGHT_RANGES)
        return 0;
    i32 count = *width * *height * (plane + 1), bits = ise_bit_count(count, *range);
    return count <= ASTC_MAX_WEIGHTS && bits >= 24 && bits <= 96;
}

static i32 find_block_mode(i32 width, i32 height, i32 range, i32 dual)
{
    for (i32 mode = 0; mode < 2048; mode++)
    {
        i32 w, h, d, r;
        if (decode_block_mode(mode, &w, &h, &d, &r) && w == width && h == height && d == dual && r == range)
            return mode;
    }
    return -1;
}

static void invert_matrix(double* matrix, i32 size, double* inverse)
{
    for (i32 i = 0; i < size; i++)
        for (i32 j = 0; j < size; j++)
            inverse[i * size + j] = i == j;
    for (i32 column = 0; column < size; column++)
    {
        i32 pivot = column;
        for (i32 row = column + 1; row < size; row++)
            if (__builtin_fabs(matrix[row * size + column]) > __builtin_fabs(matrix[pivot * size + column]))
                pivot = row;
        for (i32 k = 0; k < size; k++)
        {
            double swap = matrix[column * size + k];
            matrix[column * size + k] = matrix[pivot * size + k];
            matrix[pivot * size + k] = swap;
            swap = inverse[column * size + k];
            inverse[column * size + k] = inverse[pivot * size + k];
            inverse[pivot * size + k] = swap;
        }
        double scale = 1.0 / matrix[column * size + column];
        for (i32 k = 0; k < size; k++)
        {
            matrix[column * size + k] *= scale;
            inverse[column * size + k] *= scale;
        }
        for (i32 row = 0; row < size; row++)
        {
            if (row == column)
                continue;
            double factor = matrix[row * size + column];
            for (i32 k = 0; k < size; k++)
            {
                matrix[row * size + k] -= factor * matrix[column * size + k];
                inverse[row * size + k] -= factor * inverse[column * size + k];
            }
        }
    }
}


static void interpolation_1d(i32 block, i32 grid, double out[ASTC_MAX_SIDE][ASTC_MAX_SIDE])
{
    __builtin_memset(out, 0, sizeof(double) * ASTC_MAX_SIDE * ASTC_MAX_SIDE);
    i32 d = (1024 + block / 2) / (block - 1);
    for (i32 s = 0; s < block; s++)
    {
        i32 g = (d * s * (grid - 1) + 32) >> 6, j = g >> 4, f = g & 15;
        out[s][j] += (16 - f) / 16.0;
        if (f)
            out[s][j + 1] += f / 16.0;
    }
}

static void pseudo_inverse_1d(double interpolation[ASTC_MAX_SIDE][ASTC_MAX_SIDE], i32 block, i32 grid, double out[ASTC_MAX_SIDE][ASTC_MAX_SIDE])
{
    double normal[ASTC_MAX_SIDE * ASTC_MAX_SIDE], inverse[ASTC_MAX_SIDE * ASTC_MAX_SIDE];
    for (i32 i = 0; i < grid; i++)
        for (i32 j = 0; j < grid; j++)
        {
            double sum = 0;
            for (i32 s = 0; s < block; s++)
                sum += interpolation[s][i] * interpolation[s][j];
            normal[i * grid + j] = sum;
        }
    invert_matrix(normal, grid, inverse);
    for (i32 i = 0; i < grid; i++)
        for (i32 s = 0; s < block; s++)
        {
            double sum = 0;
            for (i32 j = 0; j < grid; j++)
                sum += inverse[i * grid + j] * interpolation[s][j];
            out[i][s] = sum;
        }
}

static inline f4 fsplat(float value) { return (f4){ value, value, value, value }; }

static inline float fsum(f4 value) { return value[0] + value[1] + value[2] + value[3]; }

static inline f4 fselect(v4 mask, f4 a, f4 b) { return (f4)(((v4)a & mask) | ((v4)b & ~mask)); }

static inline f4 fmin4(f4 a, f4 b) { return fselect((v4)(a < b), a, b); }

static inline f4 fmax4(f4 a, f4 b) { return fselect((v4)(a > b), a, b); }

static inline float least(float a, float b) { return a < b ? a : b; }

static inline float greatest(float a, float b) { return a > b ? a : b; }

static inline float clampf(float value, float low, float high) { return value < low ? low : value > high ? high : value; }

static inline float lane_of(const f4 row[2], i32 lane) { return row[lane >> 2][lane & 3]; }

static inline void set_lane(f4 row[2], i32 lane, float value) { row[lane >> 2][lane & 3] = value; }

static void initialize_large_mode(AstcLargeMode* mode, i32 block, i32 grid_width, i32 grid_height, i32 weight_range, i32 channels, i32 dual)
{
    __builtin_memset(mode, 0, sizeof(*mode));
    mode->grid_width = (u8)grid_width;
    mode->grid_height = (u8)grid_height;
    mode->weight_count = (u8)(grid_width * grid_height);
    mode->weight_range = (u8)weight_range;
    mode->channels = (u8)channels;
    mode->dual = (u8)dual;
    mode->block_mode = (u16)find_block_mode(grid_width, grid_height, weight_range, dual);
    i32 available = 128 - 17 - (dual ? 2 : 0) - ise_bit_count(mode->weight_count * (dual + 1), weight_range);
    for (i32 range = 0; range < ASTC_RANGES; range++)
        if (ise_bit_count(channels * 2, range) <= available)
            mode->endpoint_range = (u8)range;

    double along_x[ASTC_MAX_SIDE][ASTC_MAX_SIDE], along_y[ASTC_MAX_SIDE][ASTC_MAX_SIDE], inverse_x[ASTC_MAX_SIDE][ASTC_MAX_SIDE], inverse_y[ASTC_MAX_SIDE][ASTC_MAX_SIDE];
    interpolation_1d(block, grid_width, along_x);
    interpolation_1d(block, grid_height, along_y);
    pseudo_inverse_1d(along_x, block, grid_width, inverse_x);
    pseudo_inverse_1d(along_y, block, grid_height, inverse_y);
    for (i32 i = 0; i < ASTC_MAX_SIDE; i++)
        for (i32 j = 0; j < ASTC_MAX_SIDE; j++)
        {
            mode->decimate_y[i][j] = fsplat(i < grid_height && j < block ? (float)inverse_y[i][j] : 0);
            mode->decimate_x[i][j] = fsplat(i < grid_width && j < block ? (float)inverse_x[i][j] : 0);
            mode->infill_x[i][j] = fsplat(i < block && j < grid_width ? (float)along_x[i][j] : 0);
            mode->infill_y[i][j] = fsplat(i < block && j < grid_height ? (float)along_y[i][j] : 0);
        }

    i32 weight_levels = ASTC_RANGE_LEVELS[weight_range], endpoint_levels = ASTC_RANGE_LEVELS[mode->endpoint_range];
    mode->levels = (u8)weight_levels;
    float weight_step = 64.0f / (float)(weight_levels - 1), endpoint_step = 255.0f / (float)(endpoint_levels - 1);
    mode->weight_noise = (float)(block * block) * weight_step * weight_step / 12;
    mode->endpoint_noise = 0.5f * (float)(block * block * channels) * endpoint_step * endpoint_step / 12;
    i32 values[ASTC_MAX_WEIGHT_LEVELS];
    for (i32 v = 0; v < weight_levels; v++)
        values[v] = weight_unquantized(weight_range, v);
    for (i32 rank = 0; rank < weight_levels; rank++)
    {
        i32 smaller = -1;
        for (i32 v = 0; v < weight_levels; v++)
        {
            i32 below = 0;
            for (i32 other = 0; other < weight_levels; other++)
                below += values[other] < values[v];
            if (below == rank)
                smaller = v;
        }
        mode->sorted_index[rank] = (u8)smaller;
    }
    for (i32 rank = 0; rank + 1 < weight_levels; rank++)
    {
        i32 low = values[mode->sorted_index[rank]], high = values[mode->sorted_index[rank + 1]];
        mode->thresholds[rank] = (float)(low + high) * 0.5f;
        mode->steps[rank] = (float)(high - low) * (1.0f / 64);
    }
    for (i32 v = 0; v < weight_levels; v++)
        for (i32 other = 0; other < weight_levels; other++)
            if (abs32(values[other] - (64 - values[v])) < abs32(values[mode->weight_complement[v]] - (64 - values[v])))
                mode->weight_complement[v] = (u8)other;
    for (i32 v = 0; v < endpoint_levels; v++)
        mode->endpoint_value[v] = (u8)endpoint_unquantized(mode->endpoint_range, v);
    for (i32 x = 0; x < 256; x++)
        for (i32 v = 0; v < endpoint_levels; v++)
            if (abs32(mode->endpoint_value[v] - x) < abs32(mode->endpoint_value[mode->endpoint_index_of[x]] - x))
                mode->endpoint_index_of[x] = (u8)v;
}

static double square_root(double value)
{
    double root = value > 1 ? value : 1;
    for (i32 i = 0; i < 60; i++)
        root = 0.5 * (root + value / root);
    return root;
}

static double cosine(double angle)
{
    const double pi = 3.14159265358979323846;
    while (angle > pi)
        angle -= 2 * pi;
    while (angle < -pi)
        angle += 2 * pi;
    double term = 1, sum = 1;
    for (i32 n = 1; n < 30; n++)
    {
        term *= -angle * angle / (double)((2 * n - 1) * (2 * n));
        sum += term;
    }
    return sum;
}

typedef struct
{
    u8 grid_width, grid_height, weight_range, dual;
} LargeModeShape;

static const LargeModeShape ASTC_8X8_RGB_MODES[] = { { 6, 6, 2, 0 }, { 5, 8, 2, 0 }, { 8, 5, 2, 0 }, { 6, 6, 3, 0 }, { 4, 8, 3, 0 }, { 8, 3, 6, 0 }, { 6, 7, 2, 0 }, { 8, 8, 0, 0 } };
static const LargeModeShape ASTC_8X8_RGBA_MODES[] = { { 8, 8, 0, 0 }, { 8, 4, 0, 1 }, { 6, 8, 1, 0 }, { 4, 8, 0, 1 }, { 8, 5, 2, 0 }, { 2, 8, 2, 1 }, { 8, 6, 1, 0 }, { 5, 8, 2, 0 } };
static const LargeModeShape ASTC_6X6_RGB_MODES[] = { { 6, 6, 2, 0 }, { 5, 6, 4, 0 }, { 6, 6, 3, 0 }, { 6, 5, 4, 0 }, { 3, 6, 7, 0 }, { 6, 3, 8, 0 }, { 5, 5, 5, 0 }, { 4, 6, 6, 0 } };
static const LargeModeShape ASTC_6X6_RGBA_MODES[] = { { 6, 6, 2, 0 }, { 6, 5, 0, 1 }, { 4, 6, 1, 1 }, { 5, 6, 0, 1 }, { 6, 3, 2, 1 }, { 5, 6, 3, 0 }, { 6, 5, 4, 0 }, { 6, 4, 1, 1 } };

static void initialize_large_encoder(AstcLargeEncoder* encoder, i32 block, const LargeModeShape* rgb, i32 rgb_count, const LargeModeShape* rgba, i32 rgba_count)
{
    const double pi = 3.14159265358979323846;
    __builtin_memset(&encoder->mask, 0, sizeof(encoder->mask));
    encoder->block = block;
    encoder->rgb_count = rgb_count;
    encoder->rgba_count = rgba_count;
    for (i32 frequency = 0; frequency < ASTC_MAX_SIDE; frequency++)
        for (i32 position = 0; position < ASTC_MAX_SIDE; position++)
        {
            float basis = frequency < block && position < block ? (float)(square_root((frequency == 0 ? 1.0 : 2.0) / block) * cosine(pi * (2 * position + 1) * frequency / (2.0 * block))) : 0;
            encoder->dct_y[frequency][position] = fsplat(basis);
            encoder->dct_x[frequency][position] = fsplat(basis);
            set_lane(encoder->mask.row[frequency], position, frequency < block && position < block ? 1.0f : 0.0f);
        }
    for (i32 m = 0; m < rgb_count; m++)
        initialize_large_mode(&encoder->rgb[m], block, rgb[m].grid_width, rgb[m].grid_height, rgb[m].weight_range, 3, rgb[m].dual);
    for (i32 m = 0; m < rgba_count; m++)
        initialize_large_mode(&encoder->rgba[m], block, rgba[m].grid_width, rgba[m].grid_height, rgba[m].weight_range, 4, rgba[m].dual);
}

static void initialize_astc_large(void)
{
    for (u32 code = 128; code-- > 0;)
    {
        i32 quints[3];
        decode_quint_block(code, quints);
        ASTC_QUINT_BLOCK[quints[0] + 5 * quints[1] + 25 * quints[2]] = (u8)code;
    }
    initialize_large_encoder(&ASTC_8X8_ENCODER, 8, ASTC_8X8_RGB_MODES, (i32)(sizeof(ASTC_8X8_RGB_MODES) / sizeof(ASTC_8X8_RGB_MODES[0])), ASTC_8X8_RGBA_MODES, (i32)(sizeof(ASTC_8X8_RGBA_MODES) / sizeof(ASTC_8X8_RGBA_MODES[0])));
    initialize_large_encoder(&ASTC_6X6_ENCODER, 6, ASTC_6X6_RGB_MODES, (i32)(sizeof(ASTC_6X6_RGB_MODES) / sizeof(ASTC_6X6_RGB_MODES[0])), ASTC_6X6_RGBA_MODES, (i32)(sizeof(ASTC_6X6_RGBA_MODES) / sizeof(ASTC_6X6_RGBA_MODES[0])));
}

typedef struct
{
    i32 size, count;
    LargePlane value[4];
    LargePlane color_weight, color_root, alpha_weight;
    float color_total, alpha_total, sum[4];
} LargeBlock;

typedef struct
{
    float error;
    i32 swap;
    u8 endpoints[8];
    LargePlane grid[2];
} LargeFit;

static inline void transpose4(f4* a, f4* b, f4* c, f4* d)
{
    f4 t0 = __builtin_shufflevector(*a, *b, 0, 4, 1, 5), t1 = __builtin_shufflevector(*a, *b, 2, 6, 3, 7);
    f4 t2 = __builtin_shufflevector(*c, *d, 0, 4, 1, 5), t3 = __builtin_shufflevector(*c, *d, 2, 6, 3, 7);
    *a = __builtin_shufflevector(t0, t2, 0, 1, 4, 5);
    *b = __builtin_shufflevector(t0, t2, 2, 3, 6, 7);
    *c = __builtin_shufflevector(t1, t3, 0, 1, 4, 5);
    *d = __builtin_shufflevector(t1, t3, 2, 3, 6, 7);
}

static void transpose_plane(const LargePlane* in, LargePlane* out)
{
    for (i32 top = 0; top < ASTC_MAX_SIDE; top += 4)
        for (i32 half = 0; half < 2; half++)
        {
            f4 a = in->row[top][half], b = in->row[top + 1][half], c = in->row[top + 2][half], d = in->row[top + 3][half];
            transpose4(&a, &b, &c, &d);
            out->row[half * 4][top >> 2] = a;
            out->row[half * 4 + 1][top >> 2] = b;
            out->row[half * 4 + 2][top >> 2] = c;
            out->row[half * 4 + 3][top >> 2] = d;
        }
}

static void combine_rows(const LargePlane* in, const f4 matrix[ASTC_MAX_SIDE][ASTC_MAX_SIDE], i32 outputs, i32 inputs, LargePlane* out)
{
    for (i32 r = 0; r < outputs; r++)
    {
        f4 first = fsplat(0), second = fsplat(0);
        for (i32 i = 0; i < inputs; i++)
        {
            first += matrix[r][i] * in->row[i][0];
            second += matrix[r][i] * in->row[i][1];
        }
        out->row[r][0] = first;
        out->row[r][1] = second;
    }
    for (i32 r = outputs; r < ASTC_MAX_SIDE; r++)
        out->row[r][0] = out->row[r][1] = fsplat(0);
}

static void separable_transform(const LargePlane* in, const f4 rows[ASTC_MAX_SIDE][ASTC_MAX_SIDE], i32 row_outputs, const f4 columns[ASTC_MAX_SIDE][ASTC_MAX_SIDE], i32 column_outputs, i32 inputs, LargePlane* transposed)
{
    LargePlane vertical, flipped;
    combine_rows(in, rows, row_outputs, inputs, &vertical);
    transpose_plane(&vertical, &flipped);
    combine_rows(&flipped, columns, column_outputs, inputs, transposed);
}

static void principal_direction(float covariance[4][4], i32 first, i32 count, float axis[4])
{
    for (i32 c = 0; c < 4; c++)
        axis[c] = c >= first && c < first + count ? covariance[c][c] + 1e-3f : 0;
    for (i32 iteration = 0; iteration < 8; iteration++)
    {
        float next[4] = { 0, 0, 0, 0 }, largest = 1e-12f;
        for (i32 i = first; i < first + count; i++)
            for (i32 j = first; j < first + count; j++)
                next[i] += covariance[i][j] * axis[j];
        for (i32 c = first; c < first + count; c++)
            largest = __builtin_fabsf(next[c]) > largest ? __builtin_fabsf(next[c]) : largest;
        for (i32 c = first; c < first + count; c++)
            axis[c] = next[c] / largest;
    }
    if (axis[0] + axis[1] + axis[2] < 0)
        for (i32 c = 0; c < 4; c++)
            axis[c] = -axis[c];
}

static void weighted_axis(const LargeBlock* block, i32 first, i32 count, float mean[4], float axis[4])
{
    const LargePlane* roots[4] = { &block->color_root, &block->color_root, &block->color_root, &block->alpha_weight };
    for (i32 c = 0; c < 4; c++)
        mean[c] = c >= first && c < first + count ? block->sum[c] / ((c < 3 ? block->color_total : block->alpha_total) + 1e-9f) : 0;
    f4 products[4][4];
    for (i32 i = 0; i < 4; i++)
        for (i32 j = 0; j < 4; j++)
            products[i][j] = fsplat(0);
    for (i32 y = 0; y < block->size; y++)
        for (i32 h = 0; h < 2; h++)
        {
            f4 centered[4];
            for (i32 c = first; c < first + count; c++)
                centered[c] = (block->value[c].row[y][h] - fsplat(mean[c])) * roots[c]->row[y][h];
            for (i32 i = first; i < first + count; i++)
                for (i32 j = i; j < first + count; j++)
                    products[i][j] += centered[i] * centered[j];
        }
    float covariance[4][4] = { { 0 } };
    for (i32 i = first; i < first + count; i++)
        for (i32 j = i; j < first + count; j++)
            covariance[i][j] = covariance[j][i] = fsum(products[i][j]);
    principal_direction(covariance, first, count, axis);
}

static void opaque_axis(const LargeBlock* block, const LargePlane* mask, float mean[4], float axis[4])
{
    f4 first[3] = { fsplat(0), fsplat(0), fsplat(0) }, second[3][3];
    for (i32 i = 0; i < 3; i++)
        for (i32 j = 0; j < 3; j++)
            second[i][j] = fsplat(0);
    for (i32 y = 0; y < block->size; y++)
        for (i32 h = 0; h < 2; h++)
        {
            f4 inside = mask->row[y][h], values[3];
            for (i32 c = 0; c < 3; c++)
            {
                values[c] = block->value[c].row[y][h] * inside;
                first[c] += values[c];
            }
            for (i32 i = 0; i < 3; i++)
                for (i32 j = i; j < 3; j++)
                    second[i][j] += values[i] * values[j];
        }
    float count = (float)(block->size * block->size), covariance[4][4] = { { 0 } };
    for (i32 c = 0; c < 3; c++)
        mean[c] = fsum(first[c]) / count;
    mean[3] = 0;
    for (i32 i = 0; i < 3; i++)
        for (i32 j = i; j < 3; j++)
            covariance[i][j] = covariance[j][i] = fsum(second[i][j]) - mean[i] * mean[j] * count;
    principal_direction(covariance, 0, 3, axis);
}

static float rescale_plane(const LargePlane* mask, i32 size, f4 low, f4 high, LargePlane* out)
{
    float lowest = least(least(low[0], low[1]), least(low[2], low[3]));
    float highest = greatest(greatest(high[0], high[1]), greatest(high[2], high[3]));
    float span = highest - lowest, scale = span > 1e-6f ? 64 / span : 0;
    for (i32 y = 0; y < size; y++)
        for (i32 h = 0; h < 2; h++)
            out->row[y][h] = (out->row[y][h] - fsplat(lowest)) * fsplat(scale) * mask->row[y][h];
    for (i32 y = size; y < ASTC_MAX_SIDE; y++)
        out->row[y][0] = out->row[y][1] = fsplat(0);
    return span > 0 ? span : 0;
}

static float project(const LargeBlock* block, const LargePlane* mask, i32 first, i32 count, const float origin[4], const float direction[4], i32 rescale, LargePlane* out)
{
    i32 color_end = min32(first + count, 3), with_alpha = first + count > 3;
    float color_norm = 0;
    for (i32 c = first; c < color_end; c++)
        color_norm += direction[c] * direction[c];
    f4 low = fsplat(1e30f), high = fsplat(-1e30f);
    for (i32 y = 0; y < block->size; y++)
        for (i32 h = 0; h < 2; h++)
        {
            f4 along = fsplat(0), norm = fsplat(0);
            if (first < 3)
            {
                for (i32 c = first; c < color_end; c++)
                    along += (block->value[c].row[y][h] - fsplat(origin[c])) * fsplat(direction[c]);
                along *= block->color_weight.row[y][h];
                norm = block->color_weight.row[y][h] * fsplat(color_norm);
            }
            if (with_alpha)
            {
                along += block->alpha_weight.row[y][h] * (block->value[3].row[y][h] - fsplat(origin[3])) * fsplat(direction[3]);
                norm += block->alpha_weight.row[y][h] * fsplat(direction[3] * direction[3]);
            }
            v4 valid = (v4)(norm > fsplat(1e-9f));
            f4 position = fselect(valid, along / fmax4(norm, fsplat(1e-9f)), fsplat(0));
            out->row[y][h] = position;
            v4 inside = (v4)(mask->row[y][h] > fsplat(0));
            low = fmin4(low, fselect(inside, position, fsplat(1e30f)));
            high = fmax4(high, fselect(inside, position, fsplat(-1e30f)));
        }
    if (rescale)
        return rescale_plane(mask, block->size, low, high, out);
    for (i32 y = 0; y < block->size; y++)
        for (i32 h = 0; h < 2; h++)
            out->row[y][h] = fmin4(fmax4(out->row[y][h] * fsplat(64), fsplat(0)), fsplat(64)) * mask->row[y][h];
    for (i32 y = block->size; y < ASTC_MAX_SIDE; y++)
        out->row[y][0] = out->row[y][1] = fsplat(0);
    return 0;
}

static float project_opaque(const LargeBlock* block, const LargePlane* mask, const float mean[4], const float axis[4], LargePlane* out)
{
    f4 low = fsplat(1e30f), high = fsplat(-1e30f);
    for (i32 y = 0; y < block->size; y++)
        for (i32 h = 0; h < 2; h++)
        {
            f4 position = (block->value[0].row[y][h] - fsplat(mean[0])) * fsplat(axis[0]) + (block->value[1].row[y][h] - fsplat(mean[1])) * fsplat(axis[1]) + (block->value[2].row[y][h] - fsplat(mean[2])) * fsplat(axis[2]);
            out->row[y][h] = position;
            v4 inside = (v4)(mask->row[y][h] > fsplat(0));
            low = fmin4(low, fselect(inside, position, fsplat(1e30f)));
            high = fmax4(high, fselect(inside, position, fsplat(-1e30f)));
        }
    return rescale_plane(mask, block->size, low, high, out);
}

static f4 quantized_values(const AstcLargeMode* mode, f4 weights)
{
    f4 value = fsplat(0);
    for (i32 rank = 0; rank + 1 < mode->levels; rank++)
        value += fselect((v4)(weights > fsplat(mode->thresholds[rank])), fsplat(mode->steps[rank]), fsplat(0));
    return value;
}

static void quantize_weights(const AstcLargeMode* mode, i32 size, const LargePlane* ideal, LargePlane* grid, LargePlane* fraction)
{
    LargePlane values, spread, flipped;
    separable_transform(ideal, mode->decimate_y, mode->grid_height, mode->decimate_x, mode->grid_width, size, grid);
    for (i32 gx = 0; gx < ASTC_MAX_SIDE; gx++)
        for (i32 h = 0; h < 2; h++)
            values.row[gx][h] = gx < mode->grid_width ? quantized_values(mode, grid->row[gx][h]) : fsplat(0);
    combine_rows(&values, mode->infill_x, size, mode->grid_width, &spread);
    transpose_plane(&spread, &flipped);
    combine_rows(&flipped, mode->infill_y, size, mode->grid_height, fraction);
}

static void solve_endpoint(const AstcLargeMode* mode, float n, float sf, float sff, float sx, float sfx, i32* q0, i32* q1)
{
    float aa = n - 2 * sf + sff, ab = sf - sff, bb = sff, ax = sx - sfx, bx = sfx;
    float determinant = aa * bb - ab * ab, e0, e1;
    if (determinant > 1e-4f * (aa + bb) * (aa + bb) && determinant > 1e-12f)
    {
        e0 = (bb * ax - ab * bx) / determinant;
        e1 = (aa * bx - ab * ax) / determinant;
    }
    else
        e0 = e1 = n > 0 ? sx / n : 0;
    *q0 = mode->endpoint_index_of[(i32)(clampf(e0, 0, 255) + 0.5f)];
    *q1 = mode->endpoint_index_of[(i32)(clampf(e1, 0, 255) + 0.5f)];
}

static void fit_endpoints(const AstcLargeMode* mode, const LargeBlock* block, const LargePlane* color_fraction, const LargePlane* alpha_fraction, i32 q0[4], i32 q1[4])
{
    f4 sf = fsplat(0), sff = fsplat(0), sfx[4] = { fsplat(0), fsplat(0), fsplat(0), fsplat(0) }, asf = fsplat(0), asff = fsplat(0);
    for (i32 y = 0; y < block->size; y++)
        for (i32 h = 0; h < 2; h++)
        {
            f4 weight = block->color_weight.row[y][h], f = color_fraction->row[y][h], wf = weight * f;
            sf += wf;
            sff += wf * f;
            for (i32 c = 0; c < 3; c++)
                sfx[c] += wf * block->value[c].row[y][h];
            if (block->count == 4)
            {
                f4 alpha_weight = block->alpha_weight.row[y][h], af = alpha_fraction->row[y][h], awf = alpha_weight * af;
                asf += awf;
                asff += awf * af;
                sfx[3] += awf * block->value[3].row[y][h];
            }
        }
    for (i32 c = 0; c < 3; c++)
        solve_endpoint(mode, block->color_total, fsum(sf), fsum(sff), block->sum[c], fsum(sfx[c]), &q0[c], &q1[c]);
    if (block->count == 4)
        solve_endpoint(mode, block->alpha_total, fsum(asf), fsum(asff), block->sum[3], fsum(sfx[3]), &q0[3], &q1[3]);
}

static float fit_error(const AstcLargeMode* mode, const LargeBlock* block, const LargePlane* mask, const LargePlane* color_fraction, const LargePlane* alpha_fraction, const i32 q0[4], const i32 q1[4])
{
    f4 origin[4], direction[4], total = fsplat(0);
    for (i32 c = 0; c < 4; c++)
    {
        origin[c] = fsplat(mode->endpoint_value[q0[c]]);
        direction[c] = fsplat((float)(mode->endpoint_value[q1[c]] - mode->endpoint_value[q0[c]]));
    }
    for (i32 y = 0; y < block->size; y++)
        for (i32 h = 0; h < 2; h++)
        {
            f4 f = color_fraction->row[y][h], error = fsplat(0);
            if (block->count == 3)
                for (i32 c = 0; c < 3; c++)
                {
                    f4 difference = origin[c] + direction[c] * f - block->value[c].row[y][h];
                    error += difference * difference;
                }
            else
            {
                f4 decoded_alpha = origin[3] + direction[3] * alpha_fraction->row[y][h], source_alpha = block->value[3].row[y][h];
                for (i32 c = 0; c < 3; c++)
                {
                    f4 difference = ((origin[c] + direction[c] * f) * decoded_alpha - block->value[c].row[y][h] * source_alpha) * fsplat(1.0f / 255);
                    error += difference * difference;
                }
                error += (decoded_alpha - source_alpha) * (decoded_alpha - source_alpha);
            }
            total += error * mask->row[y][h];
        }
    return fsum(total);
}

static void fit_large_mode(const AstcLargeMode* mode, const LargeBlock* block, const LargePlane* mask, const LargePlane ideal_in[2], LargeFit* fit)
{
    i32 color_channels = mode->dual ? 3 : block->count;
    i32 q0[4] = { 0, 0, 0, 0 }, q1[4] = { 0, 0, 0, 0 };
    LargePlane ideal[2], fraction[2];
    ideal[0] = ideal_in[0];
    if (mode->dual)
        ideal[1] = ideal_in[1];
    for (i32 pass = 0;; pass++)
    {
        quantize_weights(mode, block->size, &ideal[0], &fit->grid[0], &fraction[0]);
        if (mode->dual)
            quantize_weights(mode, block->size, &ideal[1], &fit->grid[1], &fraction[1]);
        fit_endpoints(mode, block, &fraction[0], &fraction[mode->dual], q0, q1);
        if (pass == LARGE_REFINE)
            break;
        float origin[4], direction[4];
        for (i32 c = 0; c < 4; c++)
        {
            origin[c] = mode->endpoint_value[q0[c]];
            direction[c] = (float)(mode->endpoint_value[q1[c]] - mode->endpoint_value[q0[c]]);
        }
        project(block, mask, 0, color_channels, origin, direction, 0, &ideal[0]);
        if (mode->dual)
            project(block, mask, 3, 1, origin, direction, 0, &ideal[1]);
    }
    fit->error = fit_error(mode, block, mask, &fraction[0], &fraction[mode->dual], q0, q1);
    i32 sum0 = 0, sum1 = 0;
    for (i32 c = 0; c < 3; c++)
    {
        sum0 += mode->endpoint_value[q0[c]];
        sum1 += mode->endpoint_value[q1[c]];
    }
    fit->swap = sum1 < sum0;
    for (i32 c = 0; c < block->count; c++)
    {
        fit->endpoints[c * 2] = (u8)(fit->swap ? q1[c] : q0[c]);
        fit->endpoints[c * 2 + 1] = (u8)(fit->swap ? q0[c] : q1[c]);
    }
}

static void write_large_block(u8* restrict out, const AstcLargeMode* mode, const LargeFit* fit)
{
    u8 weights[ASTC_MAX_WEIGHTS];
    i32 stride = mode->dual + 1;
    for (i32 plane = 0; plane < stride; plane++)
        for (i32 gy = 0; gy < mode->grid_height; gy++)
            for (i32 gx = 0; gx < mode->grid_width; gx++)
            {
                float value = lane_of(fit->grid[plane].row[gx], gy);
                i32 rank = 0;
                for (i32 step = 0; step + 1 < mode->levels; step++)
                    rank += value > mode->thresholds[step];
                u8 index = mode->sorted_index[rank];
                weights[(gy * mode->grid_width + gx) * stride + plane] = fit->swap ? mode->weight_complement[index] : index;
            }
    u64 block[2] = { (u64)mode->block_mode | ((u64)(mode->channels == 4 ? 12 : 8) << 13), 0 };
    ise_write(block, 17, fit->endpoints, mode->channels * 2, mode->endpoint_range);
    u64 stream[2] = { 0, 0 };
    i32 weight_bits = ise_write(stream, 0, weights, mode->weight_count * stride, mode->weight_range);
    if (mode->dual)
    {
        i32 position = 128 - weight_bits - 2;
        put_bits(&block[0], &block[1], &position, 3, 2);
    }
    block[0] |= reverse64(stream[1]);
    block[1] |= reverse64(stream[0]);
    __builtin_memcpy(out, block, 16);
}

static void write_void_extent(u8* restrict out, u32 color)
{
    u64 low = 0xFFFFFFFFFFFFFDFCull, high = 0;
    for (i32 c = 0; c < 4; c++)
        high |= (u64)(((color >> (c * 8)) & 255) * 257) << (c * 16);
    __builtin_memcpy(out, &low, 8);
    __builtin_memcpy(out + 8, &high, 8);
}

static void choose_by_estimate(const AstcLargeEncoder* encoder, const AstcLargeMode* modes, i32 mode_count, const LargePlane* ideal, float span, i32 chosen[LARGE_FITTED_MODES])
{
    LargePlane coefficients;
    i32 size = encoder->block;
    separable_transform(ideal, encoder->dct_y, size, encoder->dct_x, size, size, &coefficients);
    float energy[ASTC_MAX_SIDE + 1][ASTC_MAX_SIDE + 1];
    for (i32 i = 0; i <= size; i++)
        energy[0][i] = energy[i][0] = 0;
    for (i32 v = 0; v < size; v++)
        for (i32 u = 0; u < size; u++)
        {
            float c = lane_of(coefficients.row[u], v);
            energy[v + 1][u + 1] = c * c + energy[v][u + 1] + energy[v + 1][u] - energy[v][u];
        }
    float total = energy[size][size], scale = span * span * (1.0f / 4096), estimates[LARGE_FITTED_MODES];
    for (i32 n = 0; n < LARGE_FITTED_MODES; n++)
    {
        chosen[n] = -1;
        estimates[n] = 1e30f;
    }
    for (i32 m = 0; m < mode_count; m++)
    {
        const AstcLargeMode* mode = &modes[m];
        float residual = total - energy[mode->grid_height][mode->grid_width];
        float estimate = scale * (residual + mode->weight_noise) + mode->endpoint_noise;
        for (i32 n = 0; n < LARGE_FITTED_MODES; n++)
            if (estimate < estimates[n])
            {
                for (i32 move = LARGE_FITTED_MODES - 1; move > n; move--)
                {
                    chosen[move] = chosen[move - 1];
                    estimates[move] = estimates[move - 1];
                }
                chosen[n] = m;
                estimates[n] = estimate;
                break;
            }
    }
}

static void prepare_block(LargeBlock* block, const LargePlane* mask, i32 opaque)
{
    f4 color_total = fsplat(0), alpha_total = fsplat(0), sums[4] = { fsplat(0), fsplat(0), fsplat(0), fsplat(0) };
    for (i32 y = 0; y < ASTC_MAX_SIDE; y++)
        for (i32 h = 0; h < 2; h++)
        {
            f4 inside = mask->row[y][h], alpha = opaque ? inside : block->value[3].row[y][h] * fsplat(1.0f / 255) * inside;
            block->color_root.row[y][h] = alpha;
            block->color_weight.row[y][h] = alpha * alpha;
            block->alpha_weight.row[y][h] = opaque ? fsplat(0) : inside;
            color_total += alpha * alpha;
            alpha_total += block->alpha_weight.row[y][h];
            for (i32 c = 0; c < 3; c++)
                sums[c] += alpha * alpha * block->value[c].row[y][h];
            sums[3] += block->alpha_weight.row[y][h] * block->value[3].row[y][h];
        }
    block->color_total = fsum(color_total);
    block->alpha_total = fsum(alpha_total);
    for (i32 c = 0; c < 4; c++)
        block->sum[c] = fsum(sums[c]);
}

static void encode_large_block(u8* restrict out, const AstcLargeEncoder* encoder, const u32* texels)
{
    LargeBlock block;
    block.size = encoder->block;
    u32 first = texels[0], same = 1, opaque = 1, transparent = 1;
    for (i32 k = 0; k < ASTC_MAX_SIDE * ASTC_MAX_SIDE; k++)
    {
        same &= texels[k] == first;
        opaque &= (texels[k] >> 24) == 255;
        transparent &= (texels[k] >> 24) == 0;
    }
    if (same || transparent)
    {
        write_void_extent(out, transparent ? 0 : first);
        return;
    }
    block.count = opaque ? 3 : 4;
    for (i32 y = 0; y < ASTC_MAX_SIDE; y++)
        for (i32 h = 0; h < 2; h++)
        {
            u4 packed;
            __builtin_memcpy(&packed, texels + y * ASTC_MAX_SIDE + h * 4, 16);
            for (i32 c = 0; c < 4; c++)
                block.value[c].row[y][h] = __builtin_convertvector((packed >> (u32)(c * 8)) & 255u, f4);
        }
    const LargePlane* mask = &encoder->mask;
    prepare_block(&block, mask, opaque);

    LargePlane single_ideal[2], dual_ideal[2];
    float mean[4], axis[4];
    if (opaque)
        opaque_axis(&block, mask, mean, axis);
    else
        weighted_axis(&block, 0, block.count, mean, axis);
    float span = opaque ? project_opaque(&block, mask, mean, axis, &single_ideal[0]) : project(&block, mask, 0, block.count, mean, axis, 1, &single_ideal[0]);
    if (!opaque)
    {
        float color_mean[4], color_axis[4], alpha_origin[4] = { 0, 0, 0, 0 }, alpha_direction[4] = { 0, 0, 0, 1 };
        weighted_axis(&block, 0, 3, color_mean, color_axis);
        project(&block, mask, 0, 3, color_mean, color_axis, 1, &dual_ideal[0]);
        project(&block, mask, 3, 1, alpha_origin, alpha_direction, 1, &dual_ideal[1]);
    }
    const AstcLargeMode* modes = opaque ? encoder->rgb : encoder->rgba;
    i32 mode_count = opaque ? encoder->rgb_count : encoder->rgba_count, chosen[ASTC_LARGE_MODES], fitted;
    if (opaque && mode_count > LARGE_FITTED_MODES)
    {
        choose_by_estimate(encoder, modes, mode_count, &single_ideal[0], span, chosen);
        fitted = LARGE_FITTED_MODES;
    }
    else
    {
        for (i32 m = 0; m < mode_count; m++)
            chosen[m] = m;
        fitted = mode_count;
    }
    LargeFit fits[2];
    i32 best = -1, best_mode = chosen[0];
    for (i32 n = 0; n < fitted && chosen[n] >= 0; n++)
    {
        const AstcLargeMode* mode = &modes[chosen[n]];
        i32 slot = best == 0 ? 1 : 0;
        fit_large_mode(mode, &block, mask, mode->dual ? dual_ideal : single_ideal, &fits[slot]);
        if (best < 0 || fits[slot].error < fits[best].error)
        {
            best = slot;
            best_mode = chosen[n];
        }
    }
    write_large_block(out, &modes[best_mode], &fits[best]);
}

static void encode_large_rows(const u32* rgba, i32 width, i32 rows, i32 valid_rows, const AstcLargeEncoder* encoder, u8* restrict out)
{
    i32 block = encoder->block, blocks_wide = (width + block - 1) / block;
    u32 texels[ASTC_MAX_SIDE * ASTC_MAX_SIDE];
    for (i32 by = 0; by * block < rows; by++)
        for (i32 bx = 0; bx < blocks_wide; bx++)
        {
            for (i32 y = 0; y < ASTC_MAX_SIDE; y++)
            {
                const u32* row = rgba + (size_t)min32(by * block + min32(y, block - 1), valid_rows - 1) * width;
                for (i32 x = 0; x < ASTC_MAX_SIDE; x++)
                    texels[y * ASTC_MAX_SIDE + x] = row[min32(bx * block + min32(x, block - 1), width - 1)];
            }
            encode_large_block(out + ((size_t)by * blocks_wide + bx) * 16, encoder, texels);
        }
}

#pragma clang fp contract(on)
