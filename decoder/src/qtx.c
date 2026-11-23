typedef unsigned char u8;
typedef unsigned short u16;
typedef short i16;
typedef unsigned int u32;
typedef int i32;
typedef long long i64;
typedef unsigned long long u64;
typedef signed char i8;
typedef __SIZE_TYPE__ size_t;

#define QTX_BUILD
#include "qtx.h"

enum { TARGET_RGBA32, TARGET_BC1, TARGET_BC3, TARGET_ETC2_RGB, TARGET_ETC2_RGBA, TARGET_ASTC_4X4, TARGET_BC7, TARGET_COUNT };
enum { TARGET_ASTC_6X6 = TARGET_COUNT, TARGET_ASTC_8X8, TARGET_ETC2_RGB_SPLIT, TARGET_ETC2_RGBA_SPLIT, API_TARGET_COUNT };
enum { PENDING_BLOCK_BYTES = 160 };
enum { FORMAT_VERSION = 1, HEADER_BYTES = 24, FIRST_ROW_ACTIVITY = 1 };
enum { TYPE_TRANSPARENT, TYPE_SMOOTH, TYPE_SOLID, TYPE_LINE };
enum { HEAD_CODE = 3, CODE_BUCKETS = 120, HEAD_TABLE = HEAD_CODE + CODE_BUCKETS, HEAD_LITERAL = HEAD_TABLE + CODE_BUCKETS, HEAD_DUAL = HEAD_LITERAL + 2, HEAD_PAIR = HEAD_DUAL + 1, HEAD_BINARY = HEAD_PAIR + 1, HEAD_COPY_LEFT = HEAD_BINARY + 2, HEAD_COPY_UP, HEAD_RUN, HEAD_SYMBOLS };
enum { MIN_RUN = 4, RUN_LENGTH_BUCKETS = 13, PAIR_SLOTS = 9, PAIR_SYMBOLS = 1 + PAIR_SLOTS * (PAIR_SLOTS - 1) };
enum { OPCODE_CONTEXTS = 9, BUCKET_SYMBOLS = 21, JOINT_SYMBOLS = 126, JOINT_ESCAPE = 125, TABLE_SYMBOLS = 8, ROW_SYMBOLS = 256, ALPHA_FINE_SYMBOLS = 16, CENTER_KINDS = 3 };
enum
{
    ALPHABET_OPCODE = 0,
    ALPHABET_RUN_LENGTH = ALPHABET_OPCODE + OPCODE_CONTEXTS,
    ALPHABET_CENTER = ALPHABET_RUN_LENGTH + 1,
    ALPHABET_TABLE_CENTER = ALPHABET_CENTER + CENTER_KINDS * 20,
    ALPHABET_EXTENT = ALPHABET_TABLE_CENTER + 5,
    ALPHABET_TABLE = ALPHABET_EXTENT + 20,
    ALPHABET_TABLE_ALPHA = ALPHABET_TABLE + 4,
    ALPHABET_ROW = ALPHABET_TABLE_ALPHA + 1,
    ALPHABET_MASK = ALPHABET_ROW + 8,
    ALPHABET_ALPHA_ROW = ALPHABET_MASK + 2,
    ALPHABET_ALPHA_FINE = ALPHABET_ALPHA_ROW + 1,
    ALPHABET_PAIR_PATTERN = ALPHABET_ALPHA_FINE + 1,
    ALPHABET_PAIR = ALPHABET_PAIR_PATTERN + 1,
    ALPHABET_COUNT = ALPHABET_PAIR + 1
};
enum { PLANE_OPCODE, PLANE_CENTER, PLANE_EXTENT, PLANE_SHAPE, PLANES, LANES = 8 };
enum { SLOT_TABLES = ALPHABET_COUNT + 1 };
enum { PROBABILITY_BITS = 11, PROBABILITY_TOTAL = 1 << PROBABILITY_BITS, PROBABILITY_MASK = PROBABILITY_TOTAL - 1 };

static const i32 BYTES_PER_BLOCK[TARGET_COUNT] = { 64, 8, 16, 8, 16, 16, 16 };

#if defined(__wasm__) || defined(QTX_WITHOUT_LIBC)
void* memset(void* target, int value, size_t size);
void* memmove(void* target, const void* source, size_t size);
void* memcpy(void* target, const void* source, size_t size);

__attribute__((no_builtin)) void* memset(void* target, int value, size_t size)
{
    u8* bytes = target;
    while (size-- > 0)
        *bytes++ = (u8)value;
    return target;
}

__attribute__((no_builtin)) void* memmove(void* target, const void* source, size_t size)
{
    u8* to = target;
    const u8* from = source;
    if (to < from)
        while (size-- > 0)
            *to++ = *from++;
    else
        while (size-- > 0)
            to[size] = from[size];
    return target;
}

__attribute__((no_builtin)) void* memcpy(void* target, const void* source, size_t size)
{
    u8* to = target;
    const u8* from = source;
    while (size-- > 0)
        *to++ = *from++;
    return target;
}
#endif

static i32 expand5_value(u32 q) { return (i32)((q << 3) | (q >> 2)); }
static i32 expand6_value(u32 q) { return (i32)((q << 2) | (q >> 4)); }
static i32 clamp255(i32 value) { return value < 0 ? 0 : value > 255 ? 255 : value; }
static i32 min32(i32 a, i32 b) { return a < b ? a : b; }
static i32 max32(i32 a, i32 b) { return a > b ? a : b; }

#include "eac_fit.h"
#include "shape_codebook.h"

static const i32 ETC_MODIFIERS[8][2] = { { 2, 8 }, { 5, 17 }, { 9, 29 }, { 13, 42 }, { 18, 60 }, { 24, 80 }, { 33, 106 }, { 47, 183 } };
static const i32 ETC_H_DISTANCES[8] = { 3, 6, 11, 16, 23, 32, 41, 64 };

static u8 QUANTIZE4[256], QUANTIZE5[256], QUANTIZE6[256], QUANTIZE7[256], ETC_TABLE_FOR[256], ETC_H_DISTANCE_FOR[256];
static u16 BC1_PAIR5[256 * 256], BC1_PAIR6[256 * 256];
static int initialized;

static i32 squared(i32 value) { return value * value; }

static void initialize_single_color(void);
static void initialize_astc_dual(void);
static void initialize_constant_blocks(void);
static void initialize_heads(void);
static void initialize_ops(void);
static void initialize_joint_vectors(void);
static void initialize_astc_large(void);

static void initialize(void)
{
    for (i32 value = 0; value < 256; value++)
    {
        i32 best5 = 0, best6 = 0;
        for (i32 q = 0; q < 64; q++)
        {
            if (squared(((q << 2) | (q >> 4)) - value) < squared(((best6 << 2) | (best6 >> 4)) - value))
                best6 = q;
            if (q < 32 && squared(((q << 3) | (q >> 2)) - value) < squared(((best5 << 3) | (best5 >> 2)) - value))
                best5 = q;
        }
        QUANTIZE5[value] = (u8)best5;
        QUANTIZE6[value] = (u8)best6;
        QUANTIZE4[value] = (u8)((value * 15 + 127) / 255);
        i32 best7 = 0;
        for (i32 q = 0; q < 128; q++)
            if (squared(((q << 1) | (q >> 6)) - value) < squared(((best7 << 1) | (best7 >> 6)) - value))
                best7 = q;
        QUANTIZE7[value] = (u8)best7;
        i32 best_table = 0, best_distance = 0;
        for (i32 table = 1; table < 8; table++)
        {
            i32 error = squared(ETC_MODIFIERS[table][0] * 3 - value) + 9 * squared(ETC_MODIFIERS[table][1] - value);
            i32 best_error = squared(ETC_MODIFIERS[best_table][0] * 3 - value) + 9 * squared(ETC_MODIFIERS[best_table][1] - value);
            if (error < best_error)
                best_table = table;
            if (squared(ETC_H_DISTANCES[table] - value) < squared(ETC_H_DISTANCES[best_distance] - value))
                best_distance = table;
        }
        ETC_TABLE_FOR[value] = (u8)best_table;
        ETC_H_DISTANCE_FOR[value] = (u8)best_distance;
    }
    for (i32 low = 0; low < 256; low++)
        for (i32 high = 0; high < 256; high++)
        {
            i32 targets[4];
            for (i32 t = 0; t < 4; t++)
                targets[t] = (low * (3 - t) + high * t + 1) / 3;
            for (i32 bits = 5; bits <= 6; bits++)
            {
                const u8* quantize = bits == 5 ? QUANTIZE5 : QUANTIZE6;
                i32 top = (1 << bits) - 1, best = 0, best_error = 1 << 30;
                i32 low_q = quantize[low], high_q = quantize[high];
                for (i32 dl = -1; dl <= 1; dl++)
                    for (i32 dh = -1; dh <= 1; dh++)
                    {
                        i32 ql = low_q + dl, qh = high_q + dh;
                        if (ql < 0 || qh < 0 || ql > top || qh > top)
                            continue;
                        i32 el = bits == 5 ? (ql << 3) | (ql >> 2) : (ql << 2) | (ql >> 4);
                        i32 eh = bits == 5 ? (qh << 3) | (qh >> 2) : (qh << 2) | (qh >> 4);
                        i32 levels[4] = { el, (2 * el + eh) / 3, (el + 2 * eh) / 3, eh };
                        i32 error = 0;
                        for (i32 t = 0; t < 4; t++)
                            error += squared(levels[t] - targets[t]);
                        if (error < best_error)
                        {
                            best_error = error;
                            best = ql | (qh << 8);
                        }
                    }
                (bits == 5 ? BC1_PAIR5 : BC1_PAIR6)[low * 256 + high] = (u16)best;
            }
        }
    initialize_single_color();
    initialize_joint_vectors();
    initialize_astc_dual();
    initialize_astc_large();
    initialize_constant_blocks();
    initialize_heads();
    initialize_ops();
    initialized = 1;
}

typedef struct
{
    const u8* symbols;
    const u32* spans;
} Alphabet;

typedef struct
{
    u32 state;
    const u8* words;
    const u8* end;
    u64 bits;
    i32 bit_count;
    const u8* raw;
    const u8* raw_end;
} Plane;

typedef struct
{
    u16 position;
    u16 first;
} Escape;

typedef struct
{
    i32 width, height, blocks_wide, blocks_high, has_alpha, alpha_step, solid_step, shared_tables, etc_split, stripe_blocks, rgba_row_base;
    i32 center_steps[4], extent_steps[4];
    u32* mip_side;
    i32 side_width, side_height;
} Header;

static i32 read_header(const u8* file, i32 length, Header* header)
{
    if (length < HEADER_BYTES || file[0] != 'Q' || file[1] != 'T' || file[2] != 'X' || file[3] != ' ' || file[8] != FORMAT_VERSION)
        return 0;
    header->width = file[4] | (file[5] << 8);
    header->height = file[6] | (file[7] << 8);
    header->has_alpha = file[9] & 1;
    for (i32 i = 0; i < 4; i++)
    {
        header->center_steps[i] = file[10 + i];
        header->extent_steps[i] = file[14 + i];
    }
    header->alpha_step = file[18];
    header->solid_step = file[22];
    header->shared_tables = (file[23] >> 1) & 1;
    header->etc_split = 0;
    header->rgba_row_base = 0;
    header->mip_side = 0;
    header->side_width = header->side_height = 0;
    header->stripe_blocks = file[19];

    header->blocks_wide = (header->width + 3) >> 2;
    header->blocks_high = (header->height + 3) >> 2;
    return header->width > 0 && header->height > 0 && header->stripe_blocks > 0;
}

enum { SLOT_FILL_BYTES = 8, SLOT_TABLE_PADDING = 64, SLOT_TABLE_BYTES = 256 * 4 + PROBABILITY_TOTAL + SLOT_TABLE_PADDING };
static size_t slot_tables_bytes(void) { return (size_t)SLOT_TABLES * SLOT_TABLE_BYTES; }

static size_t scratch_bytes(const Header* header)
{
    size_t row = (size_t)header->blocks_wide;
    return slot_tables_bytes() + (row + LANES) * (3 * 16 + 2 * 16 + 2 * 16 + 2 * 4 + 2 * 4 + 2 * sizeof(Escape) + 3 + 1 + 1 + 1 + 2 + 1 + 1 + 4 + 4 + 8 + 2 + 11 * 2) + 3 * 16 * (row + 2) + 256;
}

static u32 alphabet_size(i32 alphabet)
{
    if (alphabet < ALPHABET_RUN_LENGTH)
        return HEAD_SYMBOLS;
    if (alphabet == ALPHABET_RUN_LENGTH)
        return RUN_LENGTH_BUCKETS;
    if (alphabet >= ALPHABET_TABLE && alphabet < ALPHABET_TABLE_ALPHA)
        return TABLE_SYMBOLS;
    if (alphabet == ALPHABET_TABLE_ALPHA)
        return BUCKET_SYMBOLS;
    if (alphabet < ALPHABET_ROW)
        return (alphabet - ALPHABET_CENTER) % 5 == 0 ? JOINT_SYMBOLS : BUCKET_SYMBOLS;
    if (alphabet == ALPHABET_ALPHA_FINE)
        return ALPHA_FINE_SYMBOLS;
    if (alphabet == ALPHABET_PAIR_PATTERN)
        return CODE_BUCKETS + 1;
    if (alphabet == ALPHABET_PAIR)
        return PAIR_SYMBOLS;
    return ROW_SYMBOLS;
}

static const u32 CLASS_WEIGHTS[16] = { 0, 8, 14, 24, 42, 73, 128, 222, 388, 676, 1176, 2048, 3566, 6208, 10809, 18820 };

static void frequencies_from_classes(const u8* classes, u32 used, u32* frequencies)
{
    u32 class_count[16] = { 0 };
    for (u32 s = 0; s < used; s++)
        class_count[classes[s]]++;
    u64 total = 0;
    for (i32 k = 1; k < 16; k++)
        total += CLASS_WEIGHTS[k] * (u64)class_count[k];
    u32 value_of[16] = { 0 }, next[16] = { 0 };
    u32 count = 0;
    for (i32 k = 15; k >= 1; k--)
    {
        next[k] = count;
        count += class_count[k];
        if (class_count[k])
        {
            u64 value = CLASS_WEIGHTS[k] * (u64)PROBABILITY_TOTAL / total;
            value_of[k] = value < 1 ? 1 : (u32)value;
        }
    }
    u16 order[256];
    i32 assigned = 0;
    for (u32 s = 0; s < used; s++)
    {
        u32 k = classes[s];
        frequencies[s] = value_of[k];
        if (!k)
            continue;
        order[next[k]++] = (u16)s;
        assigned += (i32)value_of[k];
    }
    for (u32 i = 0; assigned != PROBABILITY_TOTAL; i = i + 1 == count ? 0 : i + 1)
    {
        u32 s = order[i];
        if (assigned < PROBABILITY_TOTAL)
        {
            frequencies[s]++;
            assigned++;
        }
        else if (frequencies[s] > 1)
        {
            frequencies[s]--;
            assigned--;
        }
    }
}

static void fill_slots(u8* slots, u32 symbol, u32 count)
{
    u64 pattern = symbol * 0x0101010101010101ull;
    for (u32 slot = 0; slot < count; slot += SLOT_FILL_BYTES)
        __builtin_memcpy(slots + slot, &pattern, SLOT_FILL_BYTES);
}

static const u8* read_tables(const u8* at, const u8* end, Alphabet* alphabets, u8* slot_memory)
{
    u32 used_list[ALPHABET_COUNT];
    i32 alphabet_count = ALPHABET_COUNT;
    for (i32 a = 0; a < alphabet_count; a++)
    {
        if (at >= end)
            return 0;
        u32 used = *at++;
        if (used & 0x80)
        {
            if (at >= end)
                return 0;
            used = (used & 0x7F) | ((u32)*at++ << 7);
        }
        if (used > alphabet_size(a))
            return 0;
        used_list[a] = used;
    }
    u32 nibble_index = 0;
    u32* empty_spans = (u32*)(slot_memory + (size_t)ALPHABET_COUNT * SLOT_TABLE_BYTES);
    u8* empty_slots = (u8*)(empty_spans + 256);
    empty_spans[0] = PROBABILITY_TOTAL;
    __builtin_memset(empty_slots, 0, PROBABILITY_TOTAL);
    for (i32 a = 0; a < alphabet_count; a++)
    {
        Alphabet* alphabet = &alphabets[a];
        u32 used = used_list[a];
        u8 classes[256];
        for (u32 s = 0; s < used;)
        {
            if (at + (nibble_index >> 1) >= end)
                return 0;
            u32 value = (at[nibble_index >> 1] >> ((nibble_index & 1) * 4)) & 15;
            nibble_index++;
            if (value)
            {
                classes[s++] = (u8)value;
                continue;
            }
            if (at + (nibble_index >> 1) >= end)
                return 0;
            u32 run = ((at[nibble_index >> 1] >> ((nibble_index & 1) * 4)) & 15) + 1;
            nibble_index++;
            if (s + run > used)
                return 0;
            while (run-- > 0)
                classes[s++] = 0;
        }
        u32 frequencies[256], present = 0;
        u32* own_spans = (u32*)(slot_memory + (size_t)a * SLOT_TABLE_BYTES);
        own_spans[0] = PROBABILITY_TOTAL;
        alphabet->symbols = (u8*)(own_spans + 256);
        alphabet->spans = own_spans;
        if (!used)
        {
            __builtin_memset(own_spans + 256, 0, PROBABILITY_TOTAL);
            continue;
        }
        for (u32 s = 0; s < used; s++)
            present += classes[s] != 0;
        if (!present)
            return 0;
        frequencies_from_classes(classes, used, frequencies);
        u32* spans = (u32*)(slot_memory + (size_t)a * SLOT_TABLE_BYTES);
        u8* slots = (u8*)(spans + 256);
        for (u32 s = 0, start = 0; s < used; s++)
        {
            u32 value = frequencies[s];
            spans[s] = value | (start << 16);
            fill_slots(slots + start, s, value);
            start += value;
        }
        alphabet->symbols = slots;
        alphabet->spans = spans;
    }
    return at + ((nibble_index + 1) >> 1);
}

static const u8 ZERO_WORD[2];

typedef struct
{
    u32 states[LANES];
    const u8* words[LANES];
    const u8* ends[LANES];
    u64 bits[LANES];
    i32 bit_counts[LANES];
    const u8* raws[LANES];
    const u8* raw_ends[LANES];
    const u8* limit;
} Lanes;

static __attribute__((always_inline)) inline i32 open_lanes(Lanes* lanes, const Plane* planes, const u32* counts, const u8* limit)
{
    i32 room = 1;
    lanes->limit = limit;
    for (i32 lane = 0; lane < LANES; lane++)
    {
        lanes->states[lane] = planes[lane].state;
        lanes->words[lane] = planes[lane].words;
        lanes->ends[lane] = planes[lane].end;
        lanes->bits[lane] = planes[lane].bits;
        lanes->bit_counts[lane] = planes[lane].bit_count;
        lanes->raws[lane] = planes[lane].raw;
        lanes->raw_ends[lane] = planes[lane].raw_end;
        room &= (size_t)(limit - planes[lane].words) >= (size_t)counts[lane] * 2 + 2;
    }
    return room;
}

static __attribute__((always_inline)) inline void close_lanes(const Lanes* lanes, Plane* planes)
{
    for (i32 lane = 0; lane < LANES; lane++)
    {
        planes[lane].state = lanes->states[lane];
        planes[lane].words = lanes->words[lane];
        planes[lane].bits = lanes->bits[lane];
        planes[lane].bit_count = lanes->bit_counts[lane];
        planes[lane].raw = lanes->raws[lane];
    }
}

static __attribute__((always_inline)) inline const u8* table_at(const u8* tables, u32 alphabet)
{
    return tables + (size_t)alphabet * SLOT_TABLE_BYTES;
}

typedef i32 v4 __attribute__((vector_size(16)));
typedef u8 b4 __attribute__((vector_size(4)));
typedef u32 u4 __attribute__((vector_size(16)));
typedef float f4 __attribute__((vector_size(16)));

static inline v4 vmin(v4 a, v4 b) { return __builtin_elementwise_min(a, b); }
static inline v4 vmax(v4 a, v4 b) { return __builtin_elementwise_max(a, b); }
static inline v4 vabs(v4 a) { return __builtin_elementwise_abs(a); }
static inline v4 vclamp255(v4 a) { return vmin(vmax(a, (v4){ 0, 0, 0, 0 }), (v4){ 255, 255, 255, 255 }); }
static inline i32 abs32(i32 v) { return v < 0 ? -v : v; }
static inline v4 vload(const i32* p) { v4 v; __builtin_memcpy(&v, p, 16); return v; }
static __attribute__((always_inline)) inline v4 select4(v4 mask, v4 when_set, v4 otherwise)
{
    return (when_set & mask) | (otherwise & ~mask);
}
static inline void vstore(i32* p, v4 v) { __builtin_memcpy(p, &v, 16); }
static inline void vbytes(u8* restrict out, v4 v) { b4 bytes = __builtin_convertvector(v, b4); __builtin_memcpy(out, &bytes, 4); }
static inline i32 vsum(v4 v) { return v[0] + v[1] + v[2] + v[3]; }
static inline v4 vsplat(i32 value) { return (v4){ value, value, value, value }; }

static inline v4 sums_of_four(v4 a, v4 b, v4 c, v4 d)
{
    v4 ab = __builtin_shufflevector(a, b, 0, 4, 1, 5) + __builtin_shufflevector(a, b, 2, 6, 3, 7);
    v4 cd = __builtin_shufflevector(c, d, 0, 4, 1, 5) + __builtin_shufflevector(c, d, 2, 6, 3, 7);
    return __builtin_shufflevector(ab, cd, 0, 1, 4, 5) + __builtin_shufflevector(ab, cd, 2, 3, 6, 7);
}

static const v4 BAYER_ROWS[4] = { { 0, 8, 2, 10 }, { 12, 4, 14, 6 }, { 3, 11, 1, 9 }, { 15, 7, 13, 5 } };

static inline u32 tones_from_projections(const i32 projections[4], i32 norm)
{
    const v4 xs = { 0, 1, 2, 3 }, weights = { 1, 4, 16, 64 };
    v4 first = vsplat(3 * norm - 1), second = vsplat(9 * norm - 1), third = vsplat(15 * norm - 1);
    u32 tones = 0;
    for (i32 y = 0; y < 4; y++)
    {
        i32 left = (3 - y) * projections[0] + y * projections[2];
        i32 right = (3 - y) * projections[1] + y * projections[3];
        v4 doubled = ((3 - xs) * left + xs * right) * 2;
        v4 tone = -((v4)(doubled > first) + (v4)(doubled > second) + (v4)(doubled > third));
        tones |= (u32)vsum(tone * weights) << (y * 8);
    }
    return tones;
}

static inline u32 dithered_tones_from_projections(const i32 projections[4], i32 norm)
{
    const v4 xs = { 0, 1, 2, 3 }, weights = { 1, 4, 16, 64 };
    v4 first = vsplat(3 * norm - 1), second = vsplat(9 * norm - 1), third = vsplat(15 * norm - 1);
    u32 tones = 0;
    for (i32 y = 0; y < 4; y++)
    {
        i32 left = (3 - y) * projections[0] + y * projections[2];
        i32 right = (3 - y) * projections[1] + y * projections[3];
        v4 doubled = ((3 - xs) * left + xs * right) * 2 - (BAYER_ROWS[y] * 2 - 15) * (3 * norm) / 16;
        v4 tone = -((v4)(doubled > first) + (v4)(doubled > second) + (v4)(doubled > third));
        tones |= (u32)vsum(tone * weights) << (y * 8);
    }
    return tones;
}

#define FIVE(v) v, v, v, v, v
#define TWENTY_FIVE(v) FIVE(v), FIVE(v), FIVE(v), FIVE(v), FIVE(v)
#define RED_CYCLE FIVE(-2), FIVE(-1), FIVE(0), FIVE(1), FIVE(2)
#define BLUE_ROW -2, -1, 0, 1, 2
#define BLUE_CYCLE BLUE_ROW, BLUE_ROW, BLUE_ROW, BLUE_ROW, BLUE_ROW
static const i8 JOINT_GREEN[JOINT_SYMBOLS] = { TWENTY_FIVE(-2), TWENTY_FIVE(-1), TWENTY_FIVE(0), TWENTY_FIVE(1), TWENTY_FIVE(2), 0 };
static const i8 JOINT_RED[JOINT_SYMBOLS] = { RED_CYCLE, RED_CYCLE, RED_CYCLE, RED_CYCLE, RED_CYCLE, 0 };
static const i8 JOINT_BLUE[JOINT_SYMBOLS] = { BLUE_CYCLE, BLUE_CYCLE, BLUE_CYCLE, BLUE_CYCLE, BLUE_CYCLE, 0 };

static i32 JOINT_VECTORS[JOINT_SYMBOLS][4];

static void initialize_joint_vectors(void)
{
    for (i32 joint = 0; joint < JOINT_SYMBOLS; joint++)
    {
        JOINT_VECTORS[joint][0] = JOINT_RED[joint] + JOINT_GREEN[joint];
        JOINT_VECTORS[joint][1] = JOINT_GREEN[joint];
        JOINT_VECTORS[joint][2] = JOINT_BLUE[joint] + JOINT_GREEN[joint];
        JOINT_VECTORS[joint][3] = 0;
    }
}


typedef struct
{
    u8 low[4], high[4], center[4];
    u32 tones;
    u32 alpha_tones;
    u32 alpha_fine;
    i32 dual;
    i32 transparent;
    i32 field;
    i32 table;
    i32 base5[3];
    i32 points[16];
} Block;
_Static_assert(sizeof(Block) <= PENDING_BLOCK_BYTES, "pending block storage");

static inline v4 vload_bytes(const u8* p) { b4 bytes; __builtin_memcpy(&bytes, p, 4); return __builtin_convertvector(bytes, v4); }

static inline u32 pack_color(v4 color) { b4 bytes = __builtin_convertvector(color, b4); u32 packed; __builtin_memcpy(&packed, &bytes, 4); return packed; }

static inline u32 alpha_level(i32 low, i32 high, u32 level)
{
    return (u32)(((low * (7 - (i32)level) + high * (i32)level + 3) * 9363) >> 16);
}

static inline u32 fine_alpha_level(const Block* block, i32 pixel)
{
    return (((block->alpha_tones >> (pixel * 2)) & 3) << 1) | ((block->alpha_fine >> pixel) & 1);
}

static inline u8* rgba_block_at(u8* out, const Header* header, i32 bx, i32 by)
{
    return out + ((size_t)(by * 4 - header->rgba_row_base) * header->width + (size_t)bx * 4) * 4;
}

static void emit_rgba32_dual(u8* restrict out, const Header* header, i32 bx, i32 by, const Block* block, const u32 palette[4])
{
    i32 columns = min32(4, header->width - bx * 4), rows = min32(4, header->height - by * 4);
    u8* first = rgba_block_at(out, header, bx, by);
    for (i32 y = 0; y < rows; y++)
    {
        u8* pixel = first + (size_t)y * header->width * 4;
        for (i32 x = 0; x < columns; x++)
        {
            i32 shift = (y * 4 + x) * 2;
            u32 color = (palette[(block->tones >> shift) & 3] & 0x00FFFFFFu) | (alpha_level(block->low[3], block->high[3], fine_alpha_level(block, y * 4 + x)) << 24);
            __builtin_memcpy(pixel + x * 4, &color, 4);
        }
    }
}

static void emit_rgba32(u8* restrict out, const Header* header, i32 bx, i32 by, const Block* block)
{
    v4 low = vload_bytes(block->low), high = vload_bytes(block->high);
    u32 palette[4];
    for (i32 t = 0; t < 4; t++)
        palette[t] = pack_color(((low * (3 - t) + high * t + 1) * 21846) >> 16);
    if (block->dual)
    {
        emit_rgba32_dual(out, header, bx, by, block, palette);
        return;
    }
    u32 tones = palette[0] == palette[3] ? 0 : block->tones;
    i32 columns = min32(4, header->width - bx * 4), rows = min32(4, header->height - by * 4);
    u8* first = rgba_block_at(out, header, bx, by);
    for (i32 y = 0; y < rows; y++)
    {
        u8* pixel = first + (size_t)y * header->width * 4;
        u32 line[4];
        for (i32 x = 0; x < 4; x++)
            line[x] = palette[(tones >> ((y * 4 + x) * 2)) & 3];
        if (columns == 4)
            __builtin_memcpy(pixel, line, 16);
        else
            for (i32 x = 0; x < columns; x++)
                __builtin_memcpy(pixel + x * 4, &line[x], 4);
    }
}

static u32 field_tones_between(const Block* block, const i32 low[3], const i32 high[3])
{
    v4 low_vector = { low[0], low[1], low[2], 0 }, span = { high[0] - low[0], high[1] - low[1], high[2] - low[2], 0 };
    i32 norm = vsum(span * span);
    if (norm == 0)
        return 0;
    i32 projections[4];
    for (i32 m = 0; m < 4; m++)
        projections[m] = vsum((vload(block->points + m * 4) - low_vector) * span);
    return tones_from_projections(projections, norm);
}


static u32 quantize_down(i32 value, i32 bits) { return bits == 5 ? (u32)((value * 31) / 255) : (u32)((value * 63) / 255); }
static u32 quantize_up(i32 value, i32 bits) { return bits == 5 ? (u32)((value * 31 + 254) / 255) : (u32)((value * 63 + 254) / 255); }
static i32 expand_bits(u32 q, i32 bits) { return bits == 5 ? expand5_value(q) : expand6_value(q); }

static void write_bc1(u8* restrict out, u32 low, u32 high, u32 tones)
{
    u32 tone_low = tones & 0x55555555u, tone_high = (tones >> 1) & 0x55555555u;
    u32 indices, first = high, second = low;
    if (high == low)
        indices = 0;
    else if (high > low)
        indices = ((tone_high ^ tone_low) << 1) | (~tone_high & 0x55555555u);
    else
    {
        first = low;
        second = high;
        indices = ((tone_high ^ tone_low) << 1) | tone_high;
    }
    out[0] = (u8)first;
    out[1] = (u8)(first >> 8);
    out[2] = (u8)second;
    out[3] = (u8)(second >> 8);
    out[4] = (u8)indices;
    out[5] = (u8)(indices >> 8);
    out[6] = (u8)(indices >> 16);
    out[7] = (u8)(indices >> 24);
}

enum { BC1_SINGLE_RANGE = 2, BC1_PAIR_RANGE = 4 };
static u16 SINGLE5[256], SINGLE6[256];

static void initialize_single_color(void)
{
    for (i32 bits = 5; bits <= 6; bits++)
    {
        i32 top = (1 << bits) - 1;
        for (i32 value = 0; value < 256; value++)
        {
            i32 best = 0, best_error = 1 << 30;
            for (i32 a = 0; a <= top; a++)
                for (i32 b = a > 4 ? a - 4 : 0; b <= top && b <= a + 4; b++)
                {
                    i32 level = (2 * expand_bits((u32)a, bits) + expand_bits((u32)b, bits)) / 3;
                    i32 spread = a > b ? a - b : b - a;
                    i32 error = squared(level - value) * 64 + spread;
                    if (error < best_error)
                    {
                        best_error = error;
                        best = a | (b << 8);
                    }
                }
            (bits == 5 ? SINGLE5 : SINGLE6)[value] = (u16)best;
        }
    }
}

static __attribute__((always_inline)) inline u64 bc1_single_bits(u32 red, u32 green, u32 blue)
{
    u32 r = SINGLE5[red], g = SINGLE6[green], b = SINGLE5[blue];
    u32 near = ((r & 0xFF) << 11) | ((g & 0xFF) << 5) | (b & 0xFF);
    u32 far = ((r >> 8) << 11) | ((g >> 8) << 5) | (b >> 8);
    u32 first = near > far ? near : far, second = near > far ? far : near;
    u32 indices = near > far ? 0xAAAAAAAAu : near < far ? 0xFFFFFFFFu : 0;
    return first | second << 16 | (u64)indices << 32;
}

static __attribute__((always_inline)) inline u64 bc1_line_bits(v4 low, v4 high, u32 tones)
{
    if (low[0] == high[0] && low[1] == high[1] && low[2] == high[2])
        return bc1_single_bits((u32)low[0], (u32)low[1], (u32)low[2]);
    u32 g = BC1_PAIR6[low[1] * 256 + high[1]];
    u32 r = BC1_PAIR5[low[0] * 256 + high[0]];
    u32 b = BC1_PAIR5[low[2] * 256 + high[2]];
    u32 first = ((r & 0xFF) << 11) | ((g & 0xFF) << 5) | (b & 0xFF);
    u32 second = ((r >> 8) << 11) | ((g >> 8) << 5) | (b >> 8);
    u32 tone_low = tones & 0x55555555u, tone_high = (tones >> 1) & 0x55555555u;
    u32 swap = 0u - (u32)(second < first), distinct = 0u - (u32)(second != first);
    u32 indices = (((tone_high ^ tone_low) << 1) | (tone_high ^ (~swap & 0x55555555u))) & distinct;
    u32 larger = (second & ~swap) | (first & swap), smaller = (first & ~swap) | (second & swap);
    return larger | (smaller << 16) | ((u64)indices << 32);
}

static void emit_bc1_single(u8* restrict out, const u8* color)
{
    u64 bits = bc1_single_bits(color[0], color[1], color[2]);
    __builtin_memcpy(out, &bits, 8);
}

static void emit_bc1_smooth(u8* restrict out, const Block* block)
{
    static const i32 BITS[3] = { 5, 6, 5 };
    i32 low_color[3], high_color[3];
    u32 low = 0, high = 0;
    for (i32 c = 0; c < 3; c++)
    {
        i32 a = block->low[c], b = block->high[c];
        u32 ql = a <= b ? quantize_down(a, BITS[c]) : quantize_up(a, BITS[c]);
        u32 qh = a <= b ? quantize_up(b, BITS[c]) : quantize_down(b, BITS[c]);
        low_color[c] = expand_bits(ql, BITS[c]);
        high_color[c] = expand_bits(qh, BITS[c]);
        i32 shift = c == 0 ? 11 : c == 1 ? 5 : 0;
        low |= ql << shift;
        high |= qh << shift;
    }
    write_bc1(out, low, high, field_tones_between(block, low_color, high_color));
}

static __attribute__((always_inline)) inline void emit_bc1_color(u8* restrict out, const Block* block)
{
    if (block->low[0] == block->high[0] && block->low[1] == block->high[1] && block->low[2] == block->high[2])
    {
        emit_bc1_single(out, block->low);
        return;
    }
    if (block->field)
    {
        i32 range = max32(max32(abs32(block->high[0] - block->low[0]), abs32(block->high[1] - block->low[1])), abs32(block->high[2] - block->low[2]));
        if (range < BC1_SINGLE_RANGE)
        {
            emit_bc1_single(out, block->center);
            return;
        }
        if (range > BC1_PAIR_RANGE)
        {
            emit_bc1_smooth(out, block);
            return;
        }
    }
    u64 bits = bc1_line_bits(vload_bytes(block->low), vload_bytes(block->high), block->tones);
    __builtin_memcpy(out, &bits, 8);
}

static __attribute__((always_inline)) inline u64 bc4_bits(i32 first, i32 second, const u32 tone_index[4], u32 tones)
{
    u64 bits = 0;
    for (i32 p = 0; p < 16; p++)
        bits |= (u64)tone_index[(tones >> (p * 2)) & 3] << (p * 3);
    return (u64)first | (u64)second << 8 | bits << 16;
}

static __attribute__((always_inline)) inline u64 bc4_fine_alpha_bits(i32 low, i32 high, u32 alpha_tones, u32 alpha_fine)
{
    if (low == high)
        return (u64)low | (u64)low << 8;
    i32 descending = low > high;
    static const u8 INDEX_OF_LEVEL[8] = { 1, 7, 6, 5, 4, 3, 2, 0 };
    u64 bits = 0;
    for (i32 p = 0; p < 16; p++)
    {
        u32 level = (((alpha_tones >> (p * 2)) & 3) << 1) | ((alpha_fine >> p) & 1);
        bits |= (u64)INDEX_OF_LEVEL[descending ? 7 - level : level] << (p * 3);
    }
    return (u64)(descending ? low : high) | (u64)(descending ? high : low) << 8 | bits << 16;
}

static __attribute__((always_inline)) inline u64 bc4_alpha_bits(i32 low, i32 high, u32 tones)
{
    if (low > high)
    {
        i32 swap = low;
        low = high;
        high = swap;
        tones = ~tones;
    }
    if (low == high)
        return (u64)low | (u64)low << 8;
    i32 range = high - low;
    i32 stretched = (range * 7 + 3) / 6;
    if (low + stretched <= 255)
    {
        static const u32 upward[4] = { 1, 6, 4, 2 };
        return bc4_bits(low + stretched, low, upward, tones);
    }
    if (high - stretched >= 0)
    {
        static const u32 downward[4] = { 7, 5, 3, 0 };
        return bc4_bits(high, high - stretched, downward, tones);
    }
    i32 second_level = (low * 2 + high + 1) / 3, third_level = (low + high * 2 + 1) / 3;
    u32 ends[4] = { low * 2 < second_level ? 6u : 0u, 0, 1, (255 - high) * 2 < 255 - third_level ? 7u : 1u };
    return bc4_bits(second_level, third_level, ends, tones);
}

static __attribute__((always_inline)) inline void emit_bc4_alpha(u8* restrict out, const Block* block)
{
    u64 bits = block->dual ? bc4_fine_alpha_bits(block->low[3], block->high[3], block->alpha_tones, block->alpha_fine) : bc4_alpha_bits(block->low[3], block->high[3], block->tones);
    __builtin_memcpy(out, &bits, 8);
}

#include "transcoders.c"
#include "astc_large.c"

static u8 TRANSPARENT_BLOCKS[TARGET_COUNT][16];
static u8 EAC_CONSTANT[256][8];

static u32 dithered_field_tones(i32 channels, const Block* block)
{
    v4 lanes = { 1, 1, 1, channels == 4 };
    v4 low = vload_bytes(block->low), high = vload_bytes(block->high);
    v4 span = (high - low) * lanes;
    if (!(span[0] | span[1] | span[2] | span[3]))
        return 0;
    i32 projections[4];
    for (i32 m = 0; m < 4; m++)
        projections[m] = vsum((vload(block->points + m * 4) - low) * span);
    return dithered_tones_from_projections(projections, vsum(span * span));
}

static __attribute__((always_inline)) inline void emit_block_bytes(u8* at, i32 target, i32 has_alpha, i32 etc_split, Block* block)
{
    if (target == TARGET_BC1)
        emit_bc1_color(at, block);
    else if (target == TARGET_BC3)
    {
        emit_bc4_alpha(at, block);
        emit_bc1_color(at + 8, block);
    }
    else if (target == TARGET_ETC2_RGB)
        emit_etc_color(at, block, etc_split);
    else if (target == TARGET_ETC2_RGBA)
    {
        if (block->low[3] == block->high[3])
            __builtin_memcpy(at, EAC_CONSTANT[block->low[3]], 8);
        else
            emit_eac_alpha(at, block);
        emit_etc_color(at + 8, block, etc_split);
    }
    else if (target == TARGET_ASTC_4X4)
        emit_astc(at, block, has_alpha);
    else if (target == TARGET_BC7)
        emit_bc7(at, block, has_alpha);
}

static void initialize_constant_blocks(void)
{
    Block block;
    for (i32 c = 0; c < 4; c++)
        block.low[c] = block.high[c] = block.center[c] = 0;
    block.tones = 0;
    block.transparent = 1;
    block.field = 0;
    block.dual = 0;
    block.table = -1;
    for (i32 alpha = 0; alpha < 256; alpha++)
    {
        block.low[3] = block.high[3] = (u8)alpha;
        emit_eac_alpha(EAC_CONSTANT[alpha], &block);
    }
    block.low[3] = block.high[3] = 0;
    for (i32 target = TARGET_BC1; target < TARGET_COUNT; target++)
        emit_block_bytes(TRANSPARENT_BLOCKS[target], target, 1, 0, &block);
}

static void side_write(const Header* header, i32 bx, i32 by, const u32 colors[4])
{
    for (i32 q = 0; q < 4; q++)
    {
        i32 x = bx * 2 + (q & 1), y = by * 2 + (q >> 1);
        if (x < header->side_width && y < header->side_height)
            header->mip_side[(size_t)y * header->side_width + x] = colors[q];
    }
}

static void side_block(const Header* header, i32 bx, i32 by, const Block* block)
{
    if (block->transparent)
    {
        u32 zeros[4] = { 0, 0, 0, 0 };
        side_write(header, bx, by, zeros);
        return;
    }
    if (block->field)
    {
        u32 colors[4] = { pack_color(vload(block->points)), pack_color(vload(block->points + 4)), pack_color(vload(block->points + 8)), pack_color(vload(block->points + 12)) };
        side_write(header, bx, by, colors);
        return;
    }
    v4 low = vload_bytes(block->low), high = vload_bytes(block->high);
    u32 alpha_tones = block->dual ? block->alpha_tones : block->tones;
    u32 colors[4];
    for (i32 q = 0; q < 4; q++)
    {
        i32 first = (q >> 1) * 8 + (q & 1) * 2;
        i32 sum = 0, alpha_sum = 0;
        for (i32 i = 0; i < 4; i++)
        {
            i32 pixel = first + (i & 1) + (i >> 1) * 4;
            sum += (block->tones >> (pixel * 2)) & 3;
            alpha_sum += (alpha_tones >> (pixel * 2)) & 3;
        }
        v4 weight = { sum, sum, sum, alpha_sum };
        colors[q] = pack_color(((low * (12 - weight) + high * weight) * 5461 + 32768) >> 16);
    }
    side_write(header, bx, by, colors);
}

static __attribute__((always_inline)) inline void emit_block(u8* restrict out, i32 target, const Header* header, i32 bx, i32 by, Block* block)
{
    if (header->mip_side)
        side_block(header, bx, by, block);
    if (block->field && (target == TARGET_RGBA32 || target == TARGET_ASTC_4X4))
        block->tones = dithered_field_tones(header->has_alpha ? 4 : 3, block);
    if (target == TARGET_RGBA32)
    {
        emit_rgba32(out, header, bx, by, block);
        return;
    }
    u8* at = out + ((size_t)by * header->blocks_wide + bx) * BYTES_PER_BLOCK[target];
    if (block->transparent)
    {
        if (BYTES_PER_BLOCK[target] == 16)
            __builtin_memcpy(at, TRANSPARENT_BLOCKS[target], 16);
        else
            __builtin_memcpy(at, TRANSPARENT_BLOCKS[target], 8);
        return;
    }
    emit_block_bytes(at, target, header->has_alpha, header->etc_split, block);
}

static __attribute__((always_inline)) inline void emit_transparent(u8* restrict out, i32 target, const Header* header, i32 bx, i32 by)
{
    if (header->mip_side)
    {
        u32 zeros[4] = { 0, 0, 0, 0 };
        side_write(header, bx, by, zeros);
    }
    if (target == TARGET_RGBA32)
    {
        i32 columns = min32(4, header->width - bx * 4), rows = min32(4, header->height - by * 4);
        u8* first = rgba_block_at(out, header, bx, by);
        u32 line[4] = { 0, 0, 0, 0 };
        for (i32 y = 0; y < rows; y++)
            __builtin_memcpy(first + (size_t)y * header->width * 4, line, (size_t)columns * 4);
        return;
    }
    u8* at = out + ((size_t)by * header->blocks_wide + bx) * BYTES_PER_BLOCK[target];
    if (BYTES_PER_BLOCK[target] == 16)
        __builtin_memcpy(at, TRANSPARENT_BLOCKS[target], 16);
    else
        __builtin_memcpy(at, TRANSPARENT_BLOCKS[target], 8);
}

static void field_endpoints(i32 channels, Block* block)
{
    static const u8 PAIRS[6][2] = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 }, { 2, 3 } };
    v4 points[4];
    for (i32 m = 0; m < 4; m++)
        points[m] = vload(block->points + m * 4);
    v4 lanes = { 1, 1, 1, channels == 4 };
    v4 mask = -lanes;
    v4 d01 = vabs(points[0] - points[1]) & mask, d02 = vabs(points[0] - points[2]) & mask, d03 = vabs(points[0] - points[3]) & mask;
    v4 d12 = vabs(points[1] - points[2]) & mask, d13 = vabs(points[1] - points[3]) & mask, d23 = vabs(points[2] - points[3]) & mask;
    v4 first = sums_of_four(d01, d02, d03, d12), second = sums_of_four(d13, d23, d23, d23);
    i32 best_pair = 0, best_distance = first[0];
    i32 more = first[1] > best_distance;
    best_pair = more ? 1 : best_pair;
    best_distance = more ? first[1] : best_distance;
    more = first[2] > best_distance;
    best_pair = more ? 2 : best_pair;
    best_distance = more ? first[2] : best_distance;
    more = first[3] > best_distance;
    best_pair = more ? 3 : best_pair;
    best_distance = more ? first[3] : best_distance;
    more = second[0] > best_distance;
    best_pair = more ? 4 : best_pair;
    best_distance = more ? second[0] : best_distance;
    best_pair = second[1] > best_distance ? 5 : best_pair;
    v4 low = points[PAIRS[best_pair][0]], high = points[PAIRS[best_pair][1]];
    v4 span = (high - low) * lanes;
    block->tones = 0;
    v4 magnitude = vmax(span, -span);
    if (max32(max32(magnitude[0], magnitude[1]), max32(magnitude[2], magnitude[3])) == 0)
    {
        v4 mean = (points[0] + points[1] + points[2] + points[3] + 2) >> 2;
        vbytes(block->low, mean);
        vbytes(block->high, mean);
        vbytes(block->center, mean);
        return;
    }
    vbytes(block->low, low);
    vbytes(block->high, high);
    vbytes(block->center, (low + high + 1) >> 1);
}

static void field_tones(i32 channels, Block* block)
{
    v4 lanes = { 1, 1, 1, channels == 4 };
    v4 low = vload_bytes(block->low), high = vload_bytes(block->high);
    v4 span = (high - low) * lanes;
    if (!(span[0] | span[1] | span[2] | span[3]))
    {
        block->tones = 0;
        return;
    }
    i32 projections[4];
    for (i32 m = 0; m < 4; m++)
        projections[m] = vsum((vload(block->points + m * 4) - low) * span);
    block->tones = tones_from_projections(projections, vsum(span * span));
}

static i32 needs_field_tones(i32 target, const Block* block)
{
    if (target != TARGET_BC1 && target != TARGET_BC3)
        return 0;
    i32 range = max32(max32(abs32(block->high[0] - block->low[0]), abs32(block->high[1] - block->low[1])), abs32(block->high[2] - block->low[2]));
    return range >= BC1_SINGLE_RANGE && range <= BC1_PAIR_RANGE;
}

static inline v4 unpack_color(u32 color) { b4 bytes; __builtin_memcpy(&bytes, &color, 4); return __builtin_convertvector(bytes, v4); }

static inline Block solid_block(v4 own)
{
    Block block;
    block.transparent = 0;
    block.field = 0;
    block.dual = 0;
    block.table = -1;
    block.tones = 0;
    vbytes(block.center, own);
    vbytes(block.low, own);
    vbytes(block.high, own);
    return block;
}

static __attribute__((always_inline)) inline void emit_flat_solid_block(u8* restrict out, i32 target, const Header* header, u32 own_packed, i32 row, i32 bx, const u8** repeated, u32* repeated_color)
{
    if (target != TARGET_RGBA32 && !header->mip_side && *repeated && *repeated_color == own_packed)
    {
        u8* at = out + ((size_t)row * header->blocks_wide + bx) * BYTES_PER_BLOCK[target];
        if (BYTES_PER_BLOCK[target] == 16)
            __builtin_memcpy(at, *repeated, 16);
        else
            __builtin_memcpy(at, *repeated, 8);
        return;
    }
    Block block = solid_block(unpack_color(own_packed));
    emit_block(out, target, header, bx, row, &block);
    if (target != TARGET_RGBA32)
    {
        *repeated = out + ((size_t)row * header->blocks_wide + bx) * BYTES_PER_BLOCK[target];
        *repeated_color = own_packed;
    }
}

static __attribute__((always_inline)) inline void copy_block_bytes(u8* restrict to, const u8* restrict from, i32 block_bytes)
{
    if (block_bytes == 16)
        __builtin_memcpy(to, from, 16);
    else
        __builtin_memcpy(to, from, 8);
}

static void copy_emitted_block(u8* restrict out, i32 target, const Header* header, i32 bx, i32 by, i32 source_x, i32 source_y)
{
    if (header->mip_side)
        for (i32 q = 0; q < 4; q++)
        {
            i32 x = bx * 2 + (q & 1), y = by * 2 + (q >> 1);
            i32 sx = source_x * 2 + (q & 1), sy = source_y * 2 + (q >> 1);
            if (x < header->side_width && y < header->side_height)
                header->mip_side[(size_t)y * header->side_width + x] = header->mip_side[(size_t)sy * header->side_width + sx];
        }
    if (target == TARGET_RGBA32)
    {
        i32 columns = min32(4, header->width - bx * 4), rows = min32(4, header->height - by * 4);
        u8* to = rgba_block_at(out, header, bx, by);
        const u8* from = rgba_block_at(out, header, source_x, source_y);
        for (i32 y = 0; y < rows; y++)
            __builtin_memcpy(to + (size_t)y * header->width * 4, from + (size_t)y * header->width * 4, (size_t)columns * 4);
        return;
    }
    i32 block_bytes = BYTES_PER_BLOCK[target];
    copy_block_bytes(out + ((size_t)by * header->blocks_wide + bx) * block_bytes, out + ((size_t)source_y * header->blocks_wide + source_x) * block_bytes, block_bytes);
}

typedef struct
{
    Header header;
    Alphabet alphabets[ALPHABET_COUNT];
    const u8* directory;
    const u8* stripes;
    const u8* end;
    i32 stripe_count;
} Decoding;

static i32 begin_decoding(const u8* file, i32 length, u8* scratch, Decoding* decoding, i32 inherit)
{
    if (!read_header(file, length, &decoding->header))
        return 0;
    decoding->end = file + length;
    const u8* at = file + HEADER_BYTES;
    if (decoding->header.shared_tables ? !inherit : !(at = read_tables(at, decoding->end, decoding->alphabets, scratch)))
        return -2;
    decoding->stripe_count = (decoding->header.blocks_high + decoding->header.stripe_blocks - 1) / decoding->header.stripe_blocks;
    if (at + decoding->stripe_count * 4 > decoding->end)
        return -2;
    decoding->directory = at;
    decoding->stripes = at + decoding->stripe_count * 4;
    return 1;
}

static const u8 CONTEXT_CLASS[8] = { 0, 1, 1, 2, 2, 2, 2, 2 };

static inline u32 read_u32(const u8* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((u32)p[3] << 24); }

static i32 read_varint(const u8** at, const u8* end, u32* value)
{
    u32 result = 0;
    for (i32 shift = 0; shift < 32; shift += 7)
    {
        if (*at >= end)
            return 0;
        u32 byte = *(*at)++;
        result |= (byte & 0x7F) << shift;
        if (!(byte & 0x80))
        {
            *value = result;
            return 1;
        }
    }
    return 0;
}

static i32 open_planes(const u8* stripe, const u8* stripe_end, Plane planes[PLANES * LANES])
{
    u32 sizes[PLANES * LANES][2];
    const u8* at = stripe;
    for (i32 k = 0; k < PLANES * LANES; k++)
        if (!read_varint(&at, stripe_end, &sizes[k][0]) || !read_varint(&at, stripe_end, &sizes[k][1]))
            return 0;
    for (i32 k = 0; k < PLANES * LANES; k++)
    {
        u32 size = sizes[k][0], raw_size = sizes[k][1];
        if (size > (u32)(stripe_end - at) || (size != 0 && (size < 4 || (size & 1))))
            return 0;
        Plane* plane = &planes[k];
        plane->state = size ? ((u32)(at[0] | (at[1] << 8)) << 16) | at[2] | (at[3] << 8) : 0;
        plane->words = size ? at + 4 : at;
        at += size;
        plane->end = at;
        if (raw_size > (u32)(stripe_end - at))
            return 0;
        plane->raw = at;
        plane->raw_end = at + raw_size;
        plane->bits = 0;
        plane->bit_count = 0;
        at += raw_size;
    }
    return 1;
}

static __attribute__((always_inline)) inline v4 smooth_neighbor(const i32* centers, const u8* types, i32 nx, i32 blocks_wide, v4 own)
{
    nx = max32(0, min32(nx, blocks_wide - 1));
    if (types[nx] == TYPE_TRANSPARENT)
        return own;
    return vclamp255(vload(centers + nx * 4));
}

static __attribute__((always_inline)) inline v4 divide_by_255(v4 value)
{
    return (value + 1 + (value >> 8)) >> 8;
}

static __attribute__((always_inline)) inline void transpose_colors(v4 a, v4 b, v4 c, v4 d, v4* red, v4* green, v4* blue)
{
    v4 low = __builtin_shufflevector(a, b, 0, 4, 1, 5), high = __builtin_shufflevector(c, d, 0, 4, 1, 5);
    v4 third = __builtin_shufflevector(a, b, 2, 6, 3, 7), fourth = __builtin_shufflevector(c, d, 2, 6, 3, 7);
    *red = __builtin_shufflevector(low, high, 0, 1, 4, 5);
    *green = __builtin_shufflevector(low, high, 2, 3, 6, 7);
    *blue = __builtin_shufflevector(third, fourth, 0, 1, 4, 5);
}


static void build_clamped_row(v4* clamped, const i32* centers, i32 blocks_wide)
{
    for (i32 bx = 0; bx < blocks_wide; bx++)
        clamped[bx + 1] = vclamp255(vload(centers + bx * 4));
    clamped[0] = clamped[1];
    clamped[blocks_wide + 1] = clamped[blocks_wide];
}

static __attribute__((always_inline)) inline void smooth_points4(v4 near[9][3], v4 points[4][3])
{
    for (i32 c = 0; c < 3; c++)
    {
        v4 center = near[4][c] * 9 + 8;
        points[0][c] = (center + (near[3][c] + near[1][c]) * 3 + near[0][c]) >> 4;
        points[1][c] = (center + (near[5][c] + near[1][c]) * 3 + near[2][c]) >> 4;
        points[2][c] = (center + (near[3][c] + near[7][c]) * 3 + near[6][c]) >> 4;
        points[3][c] = (center + (near[5][c] + near[7][c]) * 3 + near[8][c]) >> 4;
    }
}

static __attribute__((always_inline)) inline void smooth_farthest_pair4(v4 points[4][3], v4 low[3], v4 high[3])
{
    static const u8 PAIRS[6][2] = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 }, { 2, 3 } };
    v4 best = vsplat(0);
    for (i32 pair = 0; pair < 6; pair++)
    {
        i32 a = PAIRS[pair][0], b = PAIRS[pair][1];
        v4 distance = vabs(points[a][0] - points[b][0]) + vabs(points[a][1] - points[b][1]) + vabs(points[a][2] - points[b][2]);
        v4 more = pair == 0 ? vsplat(-1) : (v4)(distance > best);
        best = select4(more, distance, best);
        for (i32 c = 0; c < 3; c++)
        {
            low[c] = pair == 0 ? points[a][c] : select4(more, points[a][c], low[c]);
            high[c] = pair == 0 ? points[b][c] : select4(more, points[b][c], high[c]);
        }
    }
}

static __attribute__((always_inline)) inline void smooth_corners4(const i32 xs[4], const v4* up, const v4* middle, const v4* down, v4 points[4][3], v4 low[3], v4 high[3])
{
    v4 near[9][3];
    for (i32 k = 0; k < 9; k++)
    {
        const v4* source = (k < 3 ? up : k < 6 ? middle : down) + k % 3;
        transpose_colors(source[xs[0]], source[xs[1]], source[xs[2]], source[xs[3]], &near[k][0], &near[k][1], &near[k][2]);
    }
    smooth_points4(near, points);
    smooth_farthest_pair4(points, low, high);
}

static __attribute__((always_inline)) inline v4 tones_of_projections4(const v4 projections[4], v4 norm)
{
    v4 first = norm * 3 - 1, second = norm * 9 - 1, third = norm * 15 - 1;
    v4 tones = vsplat(0);
    v4 left_down = projections[2] - projections[0], across = projections[1] - projections[0];
    v4 row_start = (projections[0] << 4) + (projections[0] << 1), row_start_down = (left_down << 2) + (left_down << 1);
    v4 step = (across << 2) + (across << 1), step_down = (projections[3] - projections[2] - across) << 1;
    for (i32 y = 0; y < 4; y++)
    {
        v4 doubled = row_start;
        for (i32 x = 0; x < 4; x++)
        {
            v4 tone = -((v4)(doubled > first) + (v4)(doubled > second) + (v4)(doubled > third));
            tones |= tone << ((y * 4 + x) * 2);
            doubled += step;
        }
        row_start += row_start_down;
        step += step_down;
    }
    return tones;
}

static __attribute__((always_inline)) inline void bc1_quantized_blocks4(v4 points[4][3], const v4 low[3], const v4 high[3], v4* colors, v4* indices)
{
    static const i32 LEVELS[3] = { 31, 63, 31 };
    v4 quantized_low[3], quantized_high[3], expanded_low[3], span[3];
    v4 norm = vsplat(0);
    for (i32 c = 0; c < 3; c++)
    {
        v4 descending = (v4)(low[c] > high[c]) & 254;
        i32 levels = LEVELS[c];
        quantized_low[c] = divide_by_255(low[c] * levels + descending);
        quantized_high[c] = divide_by_255(high[c] * levels + (254 ^ descending));
        v4 expanded_high;
        if (c == 1)
        {
            expanded_low[c] = (quantized_low[c] << 2) | (quantized_low[c] >> 4);
            expanded_high = (quantized_high[c] << 2) | (quantized_high[c] >> 4);
        }
        else
        {
            expanded_low[c] = (quantized_low[c] << 3) | (quantized_low[c] >> 2);
            expanded_high = (quantized_high[c] << 3) | (quantized_high[c] >> 2);
        }
        span[c] = expanded_high - expanded_low[c];
        norm += span[c] * span[c];
    }
    v4 projections[4];
    for (i32 m = 0; m < 4; m++)
        projections[m] = (points[m][0] - expanded_low[0]) * span[0] + (points[m][1] - expanded_low[1]) * span[1] + (points[m][2] - expanded_low[2]) * span[2];
    v4 tones = tones_of_projections4(projections, norm);
    v4 low565 = (quantized_low[0] << 11) | (quantized_low[1] << 5) | quantized_low[2];
    v4 high565 = (quantized_high[0] << 11) | (quantized_high[1] << 5) | quantized_high[2];
    v4 tone_low = tones & 0x55555555, tone_high = (tones >> 1) & 0x55555555;
    v4 swap = (v4)(high565 < low565), distinct = (v4)(high565 != low565);
    *indices = (((tone_high ^ tone_low) << 1) | (tone_high ^ (~swap & 0x55555555))) & distinct;
    *colors = select4(swap, low565, high565) | (select4(swap, high565, low565) << 16);
}

static __attribute__((noinline)) void emit_smooth_bc1_opaque(u64* restrict blocks, const Header* header, i32 row, const u16* list, u32 count, const v4* up, const v4* middle, const v4* down)
{
    for (u32 n = 0; n < count; n += 4)
    {
        i32 xs[4];
        for (i32 j = 0; j < 4; j++)
            xs[j] = list[n + j < count ? n + j : count - 1];
        v4 points[4][3], low[3], high[3], colors, indices;
        smooth_corners4(xs, up, middle, down, points, low, high);
        bc1_quantized_blocks4(points, low, high, &colors, &indices);
        u64* at = blocks + (size_t)row * header->blocks_wide;
        for (i32 j = 0; j < 4; j++)
            at[xs[j]] = (u32)colors[j] | ((u64)(u32)indices[j] << 32);
        if (header->mip_side)
        {
            v4 packed[4];
            for (i32 m = 0; m < 4; m++)
                packed[m] = points[m][0] | (points[m][1] << 8) | (points[m][2] << 16) | (i32)0xFF000000u;
            for (i32 j = 0; j < 4; j++)
            {
                u32 side[4] = { (u32)packed[0][j], (u32)packed[1][j], (u32)packed[2][j], (u32)packed[3][j] };
                side_write(header, xs[j], row, side);
            }
        }
    }
}

static __attribute__((always_inline)) inline void bc7_opaque_block(u64* restrict at, v4 low, v4 high, u32 tones)
{
    u32 flip = 0u - ((tones >> 1) & 1);
    v4 mask = vsplat((i32)flip);
    v4 first = select4(mask, high, low) >> 1, second = select4(mask, low, high) >> 1;
    tones ^= flip;
    u64 bits_low = (1ull << 6) | (u64)first[0] << 7 | (u64)second[0] << 14 | (u64)first[1] << 21 | (u64)second[1] << 28 | (u64)first[2] << 35 | (u64)second[2] << 42 | 127ull << 49 | 127ull << 56 | 1ull << 63;
    u64 spread = tones;
    spread = (spread | (spread << 16)) & 0x0000FFFF0000FFFFull;
    spread = (spread | (spread << 8)) & 0x00FF00FF00FF00FFull;
    spread = (spread | (spread << 4)) & 0x0F0F0F0F0F0F0F0Full;
    spread = (spread | (spread << 2)) & 0x3333333333333333ull;
    spread |= spread << 2;
    u64 bits_high = 1 | (((spread & 7) | ((spread >> 4) << 3)) << 1);
    __builtin_memcpy(at, &bits_low, 8);
    __builtin_memcpy(at + 1, &bits_high, 8);
}

static __attribute__((always_inline)) inline void astc_opaque_block(u64* restrict at, v4 low, v4 high, u32 tones)
{
    v4 swap = vsplat(0 - (high[0] + high[1] + high[2] < low[0] + low[1] + low[2]));
    v4 first = select4(swap, high, low), second = select4(swap, low, high);
    tones ^= (u32)swap[0];
    u64 values = (u64)first[0] | (u64)second[0] << 8 | (u64)first[1] << 16 | (u64)second[1] << 24 | (u64)first[2] << 32 | (u64)second[2] << 40;
    u64 bits_low = 0x042ull | (8ull << 13) | values << 17;
    u64 bits_high = (u64)reverse32(tones) << 32 | values >> 47;
    __builtin_memcpy(at, &bits_low, 8);
    __builtin_memcpy(at + 1, &bits_high, 8);
}

static __attribute__((noinline)) void emit_wide_lines_opaque(u64* restrict blocks, i32 target, const u16* restrict list, u32 count, const i32* restrict centers, const i32* restrict extents, const u32* restrict patterns)
{
    for (u32 n = 0; n < count; n++)
    {
        i32 bx = list[n];
        v4 center = vload(centers + bx * 4), extent = vload(extents + bx * 4);
        v4 low = vclamp255(center - extent), high = vclamp255(center + extent);
        if (target == TARGET_BC7)
            bc7_opaque_block(blocks + bx * 2, low, high, patterns[bx]);
        else
            astc_opaque_block(blocks + bx * 2, low, high, patterns[bx]);
    }
}

static __attribute__((always_inline)) inline void etc_opaque_line(u8* restrict out, v4 low, v4 high, v4 center, u32 tones)
{
    v4 extent = high - low;
    i32 sum = extent[0] + extent[1] + extent[2];
    i32 magnitude = sum < 0 ? -sum : sum;
    i32 table = ETC_TABLE_FOR[min32(255, (magnitude + 3) / 6)];
    u32 lo16 = compact_even_bits(tones), hi16 = compact_even_bits(tones >> 1);
    u32 masks[4] = { ~(lo16 | hi16) & 0xFFFF, lo16 & ~hi16, hi16 & ~lo16, lo16 & hi16 };
    v4 deviation = vabs(extent * 3 - sum);
    i32 chroma = max32(max32(deviation[0], deviation[1]), deviation[2]);
    v4 low_vector = low & (v4){ -1, -1, -1, 0 }, high_vector = high & (v4){ -1, -1, -1, 0 };
    v4 palette[4] = { low_vector, ((low_vector * 2 + high_vector + 1) * 21846) >> 16, ((low_vector + high_vector * 2 + 1) * 21846) >> 16, high_vector };
    i32 counts[4];
    for (i32 t = 0; t < 4; t++)
        counts[t] = tone_count(masks[t]);
    v4 down = divide_by_255(center * 31);
    v4 up = vmin(down + 1, vsplat(31));
    v4 expanded_down = (down << 3) | (down >> 2), expanded_up = (up << 3) | (up >> 2);
    i32 a = ETC_MODIFIERS[table][0], b = ETC_MODIFIERS[table][1];
    i32 modifiers[4] = { -b, -a, a, b };
    if (sum < 0)
    {
        modifiers[0] = b;
        modifiers[1] = a;
        modifiers[2] = -a;
        modifiers[3] = -b;
    }
    v4 take_up;
    if (chroma <= 3 * 24 && !etc_rounding_clamps(expanded_down, expanded_up, b))
        take_up = etc_rounding_up_linear(expanded_down, expanded_up, palette, counts, modifiers);
    else
    {
        i32 best_error;
        take_up = etc_rounding_up_exact(expanded_down, expanded_up, palette, counts, modifiers, &best_error);
        if (chroma > 3 * 24 && etc_h_mode_below(out, palette, masks, counts, magnitude, best_error))
            return;
    }
    v4 base = (up & take_up) | (down & ~take_up);
    u32 msb_rows = sum >= 0 ? ~hi16 : hi16;
    u32 lsb_rows = ~(hi16 ^ lo16);
    u64 indices = etc_index_bits(msb_rows, lsb_rows);
    u64 head = ((u64)(base[0] << 3) << 56) | ((u64)(base[1] << 3) << 48) | ((u64)(base[2] << 3) << 40) | ((u64)((table << 5) | (table << 2) | 2) << 32);
    write_big_endian(out, head | indices);
}

static __attribute__((noinline)) void emit_etc_lines_opaque(u8* restrict out, i32 target, const u16* restrict list, u32 count, const i32* restrict centers, const i32* restrict extents,
    const u32* restrict patterns, const u8* restrict tables, const u32* restrict fives)
{
    i32 block_bytes = target == TARGET_ETC2_RGBA ? 16 : 8;
    for (u32 n = 0; n < count; n++)
    {
        i32 bx = list[n];
        u8* at = out + (size_t)bx * block_bytes;
        if (block_bytes == 16)
        {
            __builtin_memcpy(at, EAC_CONSTANT[255], 8);
            at += 8;
        }
        u32 tones = patterns[bx];
        if (tables[bx] != 0xFF)
        {
            u32 lo16 = compact_even_bits(tones), hi16 = compact_even_bits(tones >> 1), base = fives[bx], table = tables[bx];
            u64 head = ((u64)((base & 0xFF) << 3) << 56) | ((u64)(((base >> 8) & 0xFF) << 3) << 48) | ((u64)((base >> 16) << 3) << 40) | ((u64)((table << 5) | (table << 2) | 2) << 32);
            write_big_endian(at, head | etc_index_bits(~hi16, ~(hi16 ^ lo16)));
            continue;
        }
        v4 center = vload(centers + bx * 4), extent = vload(extents + bx * 4);
        v4 low = vclamp255(center - extent), high = vclamp255(center + extent);
        if (low[0] == high[0] && low[1] == high[1] && low[2] == high[2])
        {
            u8 color[4] = { (u8)low[0], (u8)low[1], (u8)low[2], 255 };
            emit_etc_planar_constant(at, color);
            continue;
        }
        etc_opaque_line(at, low, high, vclamp255(center), tones);
    }
}

static __attribute__((noinline)) void emit_smooth_wide_opaque(u64* restrict blocks, i32 target, i32 row, i32 blocks_wide, const u16* list, u32 count, const v4* up, const v4* middle, const v4* down)
{
    i32 dithered = target == TARGET_ASTC_4X4;
    for (u32 n = 0; n < count; n += 4)
    {
        i32 xs[4];
        for (i32 j = 0; j < 4; j++)
            xs[j] = list[n + j < count ? n + j : count - 1];
        v4 points[4][3], low[3], high[3], span[3];
        smooth_corners4(xs, up, middle, down, points, low, high);
        v4 norm = vsplat(0);
        for (i32 c = 0; c < 3; c++)
        {
            span[c] = high[c] - low[c];
            norm += span[c] * span[c];
        }
        v4 projections[4];
        for (i32 m = 0; m < 4; m++)
            projections[m] = (points[m][0] - low[0]) * span[0] + (points[m][1] - low[1]) * span[1] + (points[m][2] - low[2]) * span[2];
        v4 first = norm * 3 - 1, second = norm * 9 - 1, third = norm * 15 - 1;
        v4 tones = vsplat(0);
        for (i32 y = 0; y < 4; y++)
        {
            v4 left = projections[0] * (3 - y) + projections[2] * y;
            v4 right = projections[1] * (3 - y) + projections[3] * y;
            for (i32 x = 0; x < 4; x++)
            {
                v4 doubled = (left * (3 - x) + right * x) * 2;
                if (dithered)
                    doubled -= vsplat(BAYER_ROWS[y][x] * 2 - 15) * (first + 1) / 16;
                v4 tone = -((v4)(doubled > first) + (v4)(doubled > second) + (v4)(doubled > third));
                tones |= tone << ((y * 4 + x) * 2);
            }
        }
        tones &= (v4)(norm != vsplat(0));
        u64* at = blocks + (size_t)row * blocks_wide * 2;
        for (i32 j = 0; j < 4; j++)
        {
            v4 block_low = { low[0][j], low[1][j], low[2][j], 255 }, block_high = { high[0][j], high[1][j], high[2][j], 255 };
            if (target == TARGET_BC7)
                bc7_opaque_block(at + xs[j] * 2, block_low, block_high, (u32)tones[j]);
            else
                astc_opaque_block(at + xs[j] * 2, block_low, block_high, (u32)tones[j]);
        }
    }
}

static __attribute__((always_inline)) inline u4 etc_planar_quantized(v4 value12, i32 multiplier, i32 rounding)
{
    return (u4)((vclamp255(divide12_rounded(value12)) * multiplier + rounding) >> 8);
}

static __attribute__((always_inline)) inline void emit_etc_planar4(u8* const at[4], v4 points[4][3])
{
    u4 o[3], h[3], v[3];
    for (i32 c = 0; c < 3; c++)
    {
        v4 slope_x6 = points[1][c] + points[3][c] - points[0][c] - points[2][c], slope_y6 = points[2][c] + points[3][c] - points[0][c] - points[1][c];
        v4 origin12 = (points[0][c] + points[1][c] + points[2][c] + points[3][c] - slope_x6 - slope_y6) * 3;
        i32 multiplier = c == 1 ? 127 : 63, rounding = c == 1 ? 128 : 129;
        o[c] = etc_planar_quantized(origin12, multiplier, rounding);
        h[c] = etc_planar_quantized(origin12 + 8 * slope_x6, multiplier, rounding);
        v[c] = etc_planar_quantized(origin12 + 8 * slope_y6, multiplier, rounding);
    }
    u4 high = (o[0] << 25) | ((o[1] >> 6) << 24) | ((o[1] & 63) << 17) | ((o[2] >> 5) << 16) | (((o[2] >> 3) & 3) << 11) | ((o[2] & 7) << 7) | ((h[0] >> 1) << 2) | 2 | (h[0] & 1);
    u4 low = (h[1] << 25) | (h[2] << 19) | (v[0] << 13) | (v[1] << 6) | v[2];
    v4 red_underflow = (v4)((high >> 27) & 15) + ((v4)((high >> 24) & 7) ^ 4) - 4 < vsplat(0);
    v4 green_underflow = (v4)((high >> 19) & 15) + ((v4)((high >> 16) & 7) ^ 4) - 4 < vsplat(0);
    v4 blue_overflow = (v4)(((high >> 11) & 3) + ((high >> 8) & 3)) >= vsplat(4);
    high |= ((u4)red_underflow & (1u << 31)) | ((u4)green_underflow & (1u << 23)) | (((u4)blue_overflow & (7u << 13)) | (~(u4)blue_overflow & (1u << 10)));
    for (i32 j = 0; j < 4; j++)
    {
        u32 words[2] = { __builtin_bswap32(high[j]), __builtin_bswap32(low[j]) };
        __builtin_memcpy(at[j], words, 8);
    }
}

static __attribute__((noinline)) void emit_smooth_etc_opaque(u8* restrict out, i32 target, i32 row, i32 blocks_wide, const u16* list, u32 count, const v4* up, const v4* middle, const v4* down)
{
    i32 block_bytes = target == TARGET_ETC2_RGBA ? 16 : 8;
    for (u32 n = 0; n < count; n += 4)
    {
        i32 xs[4];
        for (i32 j = 0; j < 4; j++)
            xs[j] = list[n + j < count ? n + j : count - 1];
        v4 points[4][3], low[3], high[3];
        smooth_corners4(xs, up, middle, down, points, low, high);
        u8* at[4];
        for (i32 j = 0; j < 4; j++)
        {
            at[j] = out + ((size_t)row * blocks_wide + xs[j]) * block_bytes;
            if (block_bytes == 16)
            {
                __builtin_memcpy(at[j], EAC_CONSTANT[255], 8);
                at[j] += 8;
            }
        }
        emit_etc_planar4(at, points);
    }
}

static __attribute__((always_inline)) inline void emit_smooth_row(u8* restrict out, i32 target, const Header* header, i32 row, const u16* list, u32 count,
    const i32* up_centers, const u8* up_types, const i32* row_centers, const u8* row_types, const i32* down_centers, const u8* down_types, i32 lazy_tones,
    const v4* up_clamped, const v4* row_clamped, const v4* down_clamped)
{
    i32 has_alpha = header->has_alpha, channels = has_alpha ? 4 : 3, blocks_wide = header->blocks_wide;
    up_centers = up_centers ? up_centers : row_centers;
    up_types = up_centers != row_centers ? up_types : row_types;
    down_types = down_centers ? down_types : row_types;
    down_centers = down_centers ? down_centers : row_centers;
    if (target == TARGET_BC1 && !has_alpha)
    {
        emit_smooth_bc1_opaque((u64*)out, header, row, list, count, up_clamped, row_clamped, down_clamped);
        return;
    }
    if ((target == TARGET_BC7 || target == TARGET_ASTC_4X4) && !has_alpha && !header->mip_side)
    {
        emit_smooth_wide_opaque((u64*)out, target, row, blocks_wide, list, count, up_clamped, row_clamped, down_clamped);
        return;
    }
    if ((target == TARGET_ETC2_RGB || target == TARGET_ETC2_RGBA) && !has_alpha && !header->mip_side)
    {
        emit_smooth_etc_opaque(out, target, row, blocks_wide, list, count, up_clamped, row_clamped, down_clamped);
        return;
    }
    for (u32 n = 0; n < count; n++)
    {
        i32 bx = list[n];
        v4 own = vclamp255(vload(row_centers + bx * 4));
        v4 up_left = smooth_neighbor(up_centers, up_types, bx - 1, blocks_wide, own), up = smooth_neighbor(up_centers, up_types, bx, blocks_wide, own), up_right = smooth_neighbor(up_centers, up_types, bx + 1, blocks_wide, own);
        v4 left = smooth_neighbor(row_centers, row_types, bx - 1, blocks_wide, own), right = smooth_neighbor(row_centers, row_types, bx + 1, blocks_wide, own);
        v4 down_left = smooth_neighbor(down_centers, down_types, bx - 1, blocks_wide, own), down = smooth_neighbor(down_centers, down_types, bx, blocks_wide, own), down_right = smooth_neighbor(down_centers, down_types, bx + 1, blocks_wide, own);
        v4 center = own * 9 + 8;
        Block block;
        block.transparent = 0;
        block.dual = 0;
        block.table = -1;
        block.tones = 0;
        block.field = 1;
        vstore(block.points, (center + (left + up) * 3 + up_left) >> 4);
        vstore(block.points + 4, (center + (right + up) * 3 + up_right) >> 4);
        vstore(block.points + 8, (center + (left + down) * 3 + down_left) >> 4);
        vstore(block.points + 12, (center + (right + down) * 3 + down_right) >> 4);
        if (!has_alpha && (target == TARGET_ETC2_RGB || target == TARGET_ETC2_RGBA))
            block.low[3] = block.high[3] = 255;
        else
        {
            field_endpoints(channels, &block);
            if (!lazy_tones || needs_field_tones(target, &block))
                field_tones(channels, &block);
        }
        emit_block(out, target, header, bx, row, &block);
    }
}

enum { OP_TRANSPARENT, OP_SMOOTH, OP_SOLID, OP_LINE, OP_COPY_LEFT, OP_COPY_UP, OP_TABLE, OP_LITERAL, OP_LITERAL_TABLE, OP_DUAL, OP_BINARY, OP_BINARY_TABLE, OP_PAIR, OP_PAIR_LITERAL, OP_INVALID, OPS = 16 };
enum { ALPHABET_CERTAIN = ALPHABET_COUNT };

static const u8 CENTER_FIRST[OPS] = { ALPHABET_CERTAIN, ALPHABET_CENTER, ALPHABET_CENTER + 20, ALPHABET_CENTER + 40, ALPHABET_CERTAIN, ALPHABET_CERTAIN, ALPHABET_TABLE_CENTER, ALPHABET_CENTER + 40,
    ALPHABET_TABLE_CENTER, ALPHABET_CENTER + 40, ALPHABET_CENTER + 40, ALPHABET_TABLE_CENTER, ALPHABET_CERTAIN, ALPHABET_CERTAIN, ALPHABET_CERTAIN, ALPHABET_CERTAIN };
static const u8 CENTER_ACTIVITY[OPS] = { 0, 5, 5, 5, 0, 0, 0, 5, 0, 5, 5, 0, 0, 0, 0, 0 };
static const u8 EXTENT_ACTIVITY[OPS] = { 0, 0, 0, 5, 0, 0, 1, 5, 1, 5, 5, 1, 0, 0, 0, 0 };
static const u8 OP_TYPES[OPS] = { TYPE_TRANSPARENT, TYPE_SMOOTH, TYPE_SOLID, TYPE_LINE, TYPE_TRANSPARENT, TYPE_TRANSPARENT, TYPE_LINE, TYPE_LINE, TYPE_LINE, TYPE_LINE, TYPE_LINE, TYPE_LINE, TYPE_LINE, TYPE_LINE };
static const u8 OP_TABLED[OPS] = { 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1 };
static const u8 OP_LINES[OPS] = { 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1 };
static const u8 OP_ROWS[OPS] = { 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1 };
static const u8 OP_CODED[OPS] = { 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1 };
static const u8 OP_BINARIES[OPS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 };
static const u8 OP_SHAPED[OPS] = { 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1 };
static const u8 EXTENT_FIRST[OPS] = { ALPHABET_CERTAIN, ALPHABET_CERTAIN, ALPHABET_CERTAIN, ALPHABET_EXTENT, ALPHABET_CERTAIN, ALPHABET_CERTAIN, ALPHABET_TABLE, ALPHABET_EXTENT,
    ALPHABET_TABLE, ALPHABET_EXTENT, ALPHABET_EXTENT, ALPHABET_TABLE, ALPHABET_CERTAIN, ALPHABET_CERTAIN, ALPHABET_CERTAIN, ALPHABET_CERTAIN };

enum { OP_INFO_TYPE_MASK = 3, OP_INFO_LINE = 2, OP_INFO_SMOOTH, OP_INFO_SOLID, OP_INFO_TABLED, OP_INFO_ROWS, OP_INFO_COPY, OP_INFO_CODED, OP_INFO_INVALID, OP_INFO_SPREAD, OP_INFO_FLAT };
static u16 OP_INFO[OPS];

static void initialize_ops(void)
{
    for (u32 op = 0; op < OPS; op++)
        OP_INFO[op] = (u16)(OP_TYPES[op] | (u32)OP_LINES[op] << OP_INFO_LINE | (u32)(op == OP_SMOOTH) << OP_INFO_SMOOTH | (u32)(op == OP_SOLID) << OP_INFO_SOLID
            | (u32)OP_TABLED[op] << OP_INFO_TABLED | (u32)OP_ROWS[op] << OP_INFO_ROWS | (u32)(op == OP_COPY_LEFT || op == OP_COPY_UP) << OP_INFO_COPY | (u32)OP_CODED[op] << OP_INFO_CODED
            | (u32)(op == OP_TRANSPARENT || op >= OP_BINARY) << OP_INFO_INVALID | (u32)(OP_LINES[op] && !OP_TABLED[op]) << OP_INFO_SPREAD | (u32)(OP_TYPES[op] <= TYPE_SOLID) << OP_INFO_FLAT);
}

static u32 HEAD_INFO[256];
enum { LEFT_CLASS_KEPT = 3, LEFT_CLASS_FROM_UP = 4 };
enum { HEAD_INFO_BITS = 16, HEAD_INFO_OP = 20, HEAD_INFO_LEFT_CLASS = 24, HEAD_INFO_CENTER = 27, HEAD_INFO_EXTENT = 28, HEAD_INFO_SHAPE = 29 };

static const u8 CODE_BITS[CODE_BUCKETS] = {
    1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9,
    10, 10, 10, 10, 10, 10, 6, 4 };

static u32 code_bits(u32 bucket) { return CODE_BITS[bucket]; }
static u16 HEAD_STARTS[256];

static void initialize_heads(void)
{
    for (u32 head = 0; head < 256; head++)
    {
        u32 line = head - HEAD_CODE < CODE_BUCKETS, table = head - HEAD_TABLE < CODE_BUCKETS;
        u32 bucket = line ? head - HEAD_CODE : table ? head - HEAD_TABLE : 0;
        u32 op = head < HEAD_CODE ? head : line ? OP_LINE : table ? OP_TABLE : head == HEAD_LITERAL ? OP_LITERAL : head == HEAD_LITERAL + 1 ? OP_LITERAL_TABLE
            : head == HEAD_DUAL ? OP_DUAL : head == HEAD_PAIR ? OP_PAIR : head == HEAD_BINARY ? OP_BINARY : head == HEAD_BINARY + 1 ? OP_BINARY_TABLE : head == HEAD_COPY_LEFT ? OP_COPY_LEFT : head == HEAD_COPY_UP ? OP_COPY_UP : OP_INVALID;
        u32 start = 0;
        for (u32 earlier = 0; earlier < bucket; earlier++)
            start += 1u << code_bits(earlier);
        u32 bits = line | table ? code_bits(bucket) : 0;
        u32 left_class = op == OP_COPY_LEFT ? LEFT_CLASS_KEPT : op == OP_COPY_UP ? LEFT_CLASS_FROM_UP : CONTEXT_CLASS[OP_TYPES[op]];
        HEAD_STARTS[head] = (u16)(line | table ? start : 0);
        HEAD_INFO[head] = HEAD_STARTS[head] | bits << HEAD_INFO_BITS | op << HEAD_INFO_OP | left_class << HEAD_INFO_LEFT_CLASS
            | (u32)(CENTER_FIRST[op] != ALPHABET_CERTAIN) << HEAD_INFO_CENTER | (u32)(EXTENT_FIRST[op] != ALPHABET_CERTAIN) << HEAD_INFO_EXTENT | (u32)OP_SHAPED[op] << HEAD_INFO_SHAPE;
    }
}

static __attribute__((always_inline)) inline u32 code_pattern(u32 code)
{
    return SHAPE_CODEBOOK[(code >> 1) & 8191] ^ (0u - (code & 1));
}

static __attribute__((always_inline)) inline u32 binary_pattern(u32 mask)
{
    u32 spread = mask & 0xFFFF;
    spread = (spread | (spread << 8)) & 0x00FF00FF;
    spread = (spread | (spread << 4)) & 0x0F0F0F0F;
    spread = (spread | (spread << 2)) & 0x33333333;
    spread = (spread | (spread << 1)) & 0x55555555;
    return spread * 3;
}

static __attribute__((always_inline)) inline u32 line_pattern(u32 op, u32 code, u32 mask)
{
    return (code_pattern(code) & (0u - OP_CODED[op])) | (binary_pattern(mask) & (0u - OP_BINARIES[op]));
}

static __attribute__((always_inline)) inline u32 lane_symbol(Lanes* restrict lanes, i32 lane, const u8* restrict tables, u32 alphabet, i32 checked)
{
    u32 state = lanes->states[lane];
    u32 slot = state & PROBABILITY_MASK;
    const u8* table = table_at(tables, alphabet);
    u32 symbol = table[256 * 4 + slot];
    u32 span = ((const u32*)table)[symbol];
    state = (span & 0xFFFF) * (state >> PROBABILITY_BITS) + slot - (span >> 16);
    const u8* at = lanes->words[lane];
    u32 available = checked ? at + 2 <= lanes->ends[lane] : 1;
    const u8* source = available ? at : ZERO_WORD;
    u32 word = source[0] | ((u32)source[1] << 8);
    u32 need = (state < (1u << 16)) & available;
    u32 mask = 0u - need;
    lanes->states[lane] = (state & ~mask) | (((state << 16) | word) & mask);
    lanes->words[lane] = at + need * 2;
    return symbol;
}

static __attribute__((always_inline)) inline u32 lane_bits(Lanes* restrict lanes, i32 lane, i32 count, i32 checked)
{
    u64 bits = lanes->bits[lane];
    i32 held = lanes->bit_counts[lane];
    const u8* at = lanes->raws[lane];
    if (!checked)
    {
        u64 word;
        __builtin_memcpy(&word, at, 8);
        bits |= word << held;
        at += (63 - held) >> 3;
        held |= 56;
    }
    else if (held < count)
    {
        if (at + 8 <= lanes->raw_ends[lane])
        {
            u64 word;
            __builtin_memcpy(&word, at, 8);
            bits |= word << held;
            i32 taken = (63 - held) >> 3;
            at += taken;
            held += taken * 8;
        }
        else
            while (held <= 56)
            {
                u64 byte = at < lanes->raw_ends[lane] ? *at : 0;
                at++;
                bits |= byte << held;
                held += 8;
            }
    }
    u32 value = (u32)(bits & ((1ull << count) - 1));
    lanes->bits[lane] = bits >> count;
    lanes->bit_counts[lane] = held - count;
    lanes->raws[lane] = at;
    return value;
}

static __attribute__((always_inline)) inline i32 lane_signed(Lanes* restrict lanes, i32 lane, u32 bucket, i32 checked)
{
    u32 nonzero = bucket != 0;
    i32 log = (i32)(bucket - nonzero) >> 1;
    i32 magnitude = ((1 << log) + (i32)lane_bits(lanes, lane, log, checked)) * (i32)nonzero;
    return (bucket - nonzero) & 1 ? -magnitude : magnitude;
}

static __attribute__((always_inline)) inline i32 lanes_have_bits(const Lanes* lanes, u32 bytes)
{
    i32 room = 1;
    for (i32 lane = 0; lane < LANES; lane++)
        room &= (size_t)(lanes->limit - lanes->raws[lane]) >= (size_t)bytes + 16;
    return room;
}

static __attribute__((always_inline)) inline void escape_pass(Lanes* restrict lanes, const u8* restrict tables, const Escape* restrict escapes, u32 escape_count, i32* restrict overrides, i32 checked)
{
    for (u32 n = 0; n < escape_count; n++)
    {
        const Escape* escape = &escapes[n];
        i32 lane = escape->position & (LANES - 1);
        i32 green = lane_signed(lanes, lane, lane_symbol(lanes, lane, tables, escape->first + 1, checked), checked);
        i32 red = lane_signed(lanes, lane, lane_symbol(lanes, lane, tables, escape->first + 2, checked), checked);
        i32 blue = lane_signed(lanes, lane, lane_symbol(lanes, lane, tables, escape->first + 3, checked), checked);
        i32* residuals = overrides + escape->position * 4;
        residuals[0] = red + green;
        residuals[1] = green;
        residuals[2] = blue + green;
    }
}

static __attribute__((always_inline)) inline u32 decode_rows(Lanes* restrict shapes, i32 lane, const u8* restrict tables, u32 above_zero, i32 checked)
{
    u32 first = lane_symbol(shapes, lane, tables, ALPHABET_ROW + above_zero, checked);
    u32 second = lane_symbol(shapes, lane, tables, ALPHABET_ROW + 2 + (first == 0), checked);
    u32 third = lane_symbol(shapes, lane, tables, ALPHABET_ROW + 4 + (second == 0), checked);
    u32 fourth = lane_symbol(shapes, lane, tables, ALPHABET_ROW + 6 + (third == 0), checked);
    return first | (second << 8) | (third << 16) | (fourth << 24);
}

static __attribute__((always_inline)) inline u32 head_pass(Lanes* restrict heads, const u8* restrict tables, const u8* restrict contexts, u8* restrict ops, u16* restrict codes,
    u16* restrict payloads, u32* restrict payload_counts, u16* restrict runs, u32* restrict run_count, i32 blocks_wide, i32 padded, i32 left_mode, i32 checked)
{
    u32 left_class = 0, center_count = 0, extent_count = 0, shape_count = 0;
    for (i32 bx = 0; bx < blocks_wide;)
    {
        i32 lane = bx & (LANES - 1);
        u32 head = lane_symbol(heads, lane, tables, contexts[bx] + (left_mode ? left_class * 3 : 0), checked);
        if (head == HEAD_RUN)
        {
            u32 bucket = lane_symbol(heads, lane, tables, ALPHABET_RUN_LENGTH, 1);
            i32 run = MIN_RUN + (bucket ? (i32)((1u << (bucket - 1)) + lane_bits(heads, lane, (i32)bucket - 1, 1)) : 0);
            u32 repeated = bx > 0 ? ops[bx - 1] : OP_INVALID;
            if (run > blocks_wide - bx || !(repeated == OP_TRANSPARENT || repeated == OP_COPY_LEFT || repeated == OP_COPY_UP))
                return 1;
            for (i32 at = bx; at < bx + run; at++)
                ops[at] = (u8)repeated;
            runs[*run_count * 2] = (u16)bx;
            runs[*run_count * 2 + 1] = (u16)run;
            ++*run_count;
            left_class = repeated == OP_TRANSPARENT ? 0 : repeated == OP_COPY_UP ? (u32)(contexts[bx + run - 1] - ALPHABET_OPCODE) : left_class;
            bx += run;
            continue;
        }
        u32 info = HEAD_INFO[head];
        u32 extra = lane_bits(heads, lane, (i32)((info >> HEAD_INFO_BITS) & 15), checked);
        codes[bx] = (u16)((info & 0xFFFF) + extra);
        ops[bx] = (u8)((info >> HEAD_INFO_OP) & 15);
        u32 kind = (info >> HEAD_INFO_LEFT_CLASS) & 7;
        left_class = kind < LEFT_CLASS_KEPT ? kind : kind == LEFT_CLASS_KEPT ? left_class : (u32)(contexts[bx] - ALPHABET_OPCODE);
        payloads[center_count] = (u16)bx;
        center_count += (info >> HEAD_INFO_CENTER) & 1;
        payloads[padded + extent_count] = (u16)bx;
        extent_count += (info >> HEAD_INFO_EXTENT) & 1;
        payloads[padded * 2 + shape_count] = (u16)bx;
        shape_count += (info >> HEAD_INFO_SHAPE) & 1;
        bx++;
    }
    payload_counts[0] = center_count;
    payload_counts[1] = extent_count;
    payload_counts[2] = shape_count;
    return 0;
}

static __attribute__((always_inline)) inline void center_pass(Lanes* restrict centers, const u8* restrict tables, const u8* restrict activities, const u8* restrict ops, u8* restrict joints, i32* restrict alphas,
    Escape* restrict escapes, u32* restrict escape_count, const u16* restrict list, u32 count, i32 has_alpha, i32 checked)
{
    u32 escaped = 0;
    for (u32 n = 0; n < count; n++)
    {
        i32 bx = list[n], lane = bx & (LANES - 1);
        u32 op = ops[bx], first = CENTER_FIRST[op] + CENTER_ACTIVITY[op] * activities[bx];
        u32 joint = lane_symbol(centers, lane, tables, first, checked);
        joint = joint < JOINT_SYMBOLS ? joint : JOINT_ESCAPE;
        joints[bx] = (u8)joint;
        escapes[escaped].position = (u16)bx;
        escapes[escaped].first = (u16)first;
        escaped += joint == JOINT_ESCAPE;
        if (has_alpha)
            alphas[bx] = lane_signed(centers, lane, lane_symbol(centers, lane, tables, first + 4, checked), checked);
    }
    *escape_count = escaped;
}

static __attribute__((always_inline)) inline void extent_pass(Lanes* restrict extents, const u8* restrict tables, const u8* restrict activities, const u8* restrict ops, u8* restrict joints, i32* restrict alphas,
    Escape* restrict escapes, u32* restrict escape_count, const u16* restrict list, u32 count, i32 has_alpha, i32 checked)
{
    u32 escaped = 0;
    for (u32 n = 0; n < count; n++)
    {
        i32 bx = list[n], lane = bx & (LANES - 1);
        u32 op = ops[bx], first = EXTENT_FIRST[op] + EXTENT_ACTIVITY[op] * activities[bx];
        u32 joint = lane_symbol(extents, lane, tables, first, checked);
        joints[bx] = (u8)joint;
        escapes[escaped].position = (u16)bx;
        escapes[escaped].first = (u16)first;
        escaped += (joint == JOINT_ESCAPE) & !OP_TABLED[op];
        if (has_alpha)
            alphas[bx] = lane_signed(extents, lane, lane_symbol(extents, lane, tables, OP_TABLED[op] ? ALPHABET_TABLE_ALPHA : first + 4, checked), checked);
    }
    *escape_count = escaped;
}

static __attribute__((always_inline)) inline void shape_pass(Lanes* restrict shapes, const u8* restrict tables, const u32* restrict up_patterns, u8* restrict ops, u16* restrict codes, u32* restrict rows,
    u32* restrict alpha_rows, u32* restrict alpha_fines, u8* restrict pair_symbols, const u16* restrict list, u32 count, i32 by, i32 checked)
{
    for (u32 n = 0; n < count; n++)
    {
        i32 bx = list[n], lane = bx & (LANES - 1);
        u32 op = ops[bx], literal = OP_ROWS[op];
        if (op == OP_PAIR)
        {
            u32 bucket = lane_symbol(shapes, lane, tables, ALPHABET_PAIR_PATTERN, checked);
            literal = bucket >= CODE_BUCKETS;
            codes[bx] = literal ? 0 : (u16)(HEAD_STARTS[HEAD_CODE + bucket] + lane_bits(shapes, lane, CODE_BITS[bucket], checked));
            ops[bx] = (u8)(literal ? OP_PAIR_LITERAL : OP_PAIR);
        }
        if (OP_BINARIES[op])
        {
            u32 low = lane_symbol(shapes, lane, tables, ALPHABET_MASK, checked);
            rows[bx] = low | (lane_symbol(shapes, lane, tables, ALPHABET_MASK + 1, checked) << 8);
        }
        if (literal)
        {
            rows[bx] = decode_rows(shapes, lane, tables, by == 0 || up_patterns[bx] == 0, checked);
            if (op == OP_DUAL)
            {
                u32 coarse = 0, fine = 0;
                for (i32 line = 0; line < 4; line++)
                    coarse |= lane_symbol(shapes, lane, tables, ALPHABET_ALPHA_ROW, checked) << (line * 8);
                for (i32 line = 0; line < 4; line++)
                    fine |= lane_symbol(shapes, lane, tables, ALPHABET_ALPHA_FINE, checked) << (line * 4);
                alpha_rows[bx] = coarse;
                alpha_fines[bx] = fine;
            }
        }
        if (op == OP_PAIR)
            pair_symbols[bx] = (u8)lane_symbol(shapes, lane, tables, ALPHABET_PAIR, checked);
    }
}

static __attribute__((noinline)) u32 decode_row_symbols(Plane* planes, const u8* restrict tables, const u8* restrict contexts, const u8* restrict activities,
    const u32* restrict up_patterns, u8* restrict ops, u16* restrict codes, u32* restrict rows, u32* restrict alpha_rows, u32* restrict alpha_fines, u8* restrict pair_symbols, u8* restrict joints, i32* restrict alphas, i32* restrict overrides, Escape* restrict escapes,
    u16* restrict payloads, u16* restrict runs, u32* restrict run_count, i32 blocks_wide, i32 lane_width, i32 padded, i32 by, i32 has_alpha, i32 left_mode, const u8* limit)
{
    Lanes heads, centers, extents, shapes;
    u32 budget[LANES], escape_counts[2] = { 0, 0 };
    for (i32 lane = 0; lane < LANES; lane++)
        budget[lane] = (u32)lane_width * 2;
    i32 room = open_lanes(&heads, planes + PLANE_OPCODE * LANES, budget, limit) & lanes_have_bits(&heads, (u32)lane_width * 2);
    i32 joint_room = open_lanes(&centers, planes + PLANE_CENTER * LANES, budget, limit);
    joint_room &= open_lanes(&extents, planes + PLANE_EXTENT * LANES, budget, limit);
    room &= joint_room;
    u32 shape_budget[LANES];
    for (i32 lane = 0; lane < LANES; lane++)
        shape_budget[lane] = (u32)lane_width * 4;
    room &= open_lanes(&shapes, planes + PLANE_SHAPE * LANES, shape_budget, limit) & lanes_have_bits(&shapes, (u32)lane_width * 2);
    room &= !has_alpha || (lanes_have_bits(&centers, (u32)lane_width * 2) & lanes_have_bits(&extents, (u32)lane_width * 2));
    u32 payload_counts[3] = { 0, 0, 0 };
    u32 invalid = !room ? head_pass(&heads, tables, contexts, ops, codes, payloads, payload_counts, runs, run_count, blocks_wide, padded, left_mode, 1)
        : left_mode ? head_pass(&heads, tables, contexts, ops, codes, payloads, payload_counts, runs, run_count, blocks_wide, padded, 1, 0)
        : head_pass(&heads, tables, contexts, ops, codes, payloads, payload_counts, runs, run_count, blocks_wide, padded, 0, 0);
    if (invalid)
        return 1;
    if (!room)
    {
        center_pass(&centers, tables, activities, ops, joints, alphas, escapes, escape_counts, payloads, payload_counts[0], has_alpha, 1);
        extent_pass(&extents, tables, activities, ops, joints + padded, alphas + padded, escapes + padded, escape_counts + 1, payloads + padded, payload_counts[1], has_alpha, 1);
        shape_pass(&shapes, tables, up_patterns, ops, codes, rows, alpha_rows, alpha_fines, pair_symbols, payloads + padded * 2, payload_counts[2], by, 1);
    }
    else if (has_alpha)
    {
        center_pass(&centers, tables, activities, ops, joints, alphas, escapes, escape_counts, payloads, payload_counts[0], 1, 0);
        extent_pass(&extents, tables, activities, ops, joints + padded, alphas + padded, escapes + padded, escape_counts + 1, payloads + padded, payload_counts[1], 1, 0);
        shape_pass(&shapes, tables, up_patterns, ops, codes, rows, alpha_rows, alpha_fines, pair_symbols, payloads + padded * 2, payload_counts[2], by, 0);
    }
    else
    {
        center_pass(&centers, tables, activities, ops, joints, alphas, escapes, escape_counts, payloads, payload_counts[0], 0, 0);
        extent_pass(&extents, tables, activities, ops, joints + padded, alphas + padded, escapes + padded, escape_counts + 1, payloads + padded, payload_counts[1], 0, 0);
        shape_pass(&shapes, tables, up_patterns, ops, codes, rows, alpha_rows, alpha_fines, pair_symbols, payloads + padded * 2, payload_counts[2], by, 0);
    }
    close_lanes(&heads, planes + PLANE_OPCODE * LANES);
    close_lanes(&shapes, planes + PLANE_SHAPE * LANES);
    Lanes* joint_lanes[2] = { &centers, &extents };
    for (i32 plane = 0; plane < 2; plane++)
    {
        Lanes* lanes = joint_lanes[plane];
        u32 count = escape_counts[plane];
        i32 escape_room = lanes_have_bits(lanes, count * 12);
        for (i32 lane = 0; lane < LANES; lane++)
            escape_room &= (size_t)(lanes->limit - lanes->words[lane]) >= (size_t)count * 6 + 2;
        if (escape_room)
            escape_pass(lanes, tables, escapes + plane * padded, count, overrides + plane * padded * 4, 0);
        else
            escape_pass(lanes, tables, escapes + plane * padded, count, overrides + plane * padded * 4, 1);
    }
    close_lanes(&centers, planes + PLANE_CENTER * LANES);
    close_lanes(&extents, planes + PLANE_EXTENT * LANES);
    return invalid;
}

static __attribute__((always_inline)) inline v4 residual_of(u32 joint, const i32* override, i32 alpha, i32 has_alpha)
{
    v4 residual = select4(vsplat(0 - (joint == JOINT_ESCAPE)), vload(override), vload(JOINT_VECTORS[joint < JOINT_SYMBOLS ? joint : 0]));
    residual[3] = has_alpha ? alpha : residual[3];
    return residual;
}

static const i32 TABLE_EXTENTS[TABLE_SYMBOLS] = { 8, 17, 29, 42, 60, 80, 106, 183 };

static __attribute__((noinline)) void emit_bc1_lines(u64* restrict blocks, const u16* restrict list, u32 count, const i32* restrict centers, const i32* restrict extents, const u32* restrict patterns)
{
    for (u32 n = 0; n < count; n++)
    {
        i32 bx = list[n];
        v4 center = vload(centers + bx * 4), extent = vload(extents + bx * 4);
        blocks[bx] = bc1_line_bits(vclamp255(center - extent), vclamp255(center + extent), patterns[bx]);
    }
}

static __attribute__((noinline)) void emit_bc3_lines(u64* restrict blocks, const u16* restrict list, u32 count, const i32* restrict centers, const i32* restrict extents, const u32* restrict patterns,
    const u8* restrict ops, const u32* restrict alpha_rows, const u32* restrict alpha_fines)
{
    for (u32 n = 0; n < count; n++)
    {
        i32 bx = list[n];
        v4 center = vload(centers + bx * 4), extent = vload(extents + bx * 4);
        v4 low = vclamp255(center - extent), high = vclamp255(center + extent);
        u32 tones = patterns[bx];
        blocks[bx * 2] = ops[bx] == OP_DUAL ? bc4_fine_alpha_bits(low[3], high[3], alpha_rows[bx], alpha_fines[bx]) : bc4_alpha_bits(low[3], high[3], tones);
        blocks[bx * 2 + 1] = bc1_line_bits(low, high, tones);
    }
}

static __attribute__((always_inline)) inline v4 vmed(v4 left, v4 up, v4 corner)
{
    return vmin(vmax(left + up - corner, vmin(left, up)), vmax(left, up));
}

static __attribute__((always_inline)) inline u32 add_pair_colors(u32* colors, u32 count, u32 type, v4 middle, v4 spread)
{
    if (type == TYPE_SOLID)
        colors[count++] = pack_color(vclamp255(middle));
    else if (type == TYPE_LINE)
    {
        colors[count++] = pack_color(vclamp255(middle - spread));
        colors[count++] = pack_color(vclamp255(middle + spread));
    }
    return count;
}

static __attribute__((always_inline)) inline u32 add_tones(u32 a, u32 b);

static __attribute__((always_inline)) inline u32 literal_pattern(u32 shape, u32 above)
{
    u32 row0 = add_tones(shape & 0xFF, above >> 24);
    u32 row1 = add_tones((shape >> 8) & 0xFF, row0);
    u32 row2 = add_tones((shape >> 16) & 0xFF, row1);
    u32 row3 = add_tones(shape >> 24, row2);
    return row0 | (row1 << 8) | (row2 << 16) | (row3 << 24);
}

static __attribute__((always_inline)) inline u32 dual_alpha_pattern(u32 alpha_shape, u32 pattern)
{
    u32 alpha_pattern = 0;
    for (i32 r = 0; r < 4; r++)
        alpha_pattern |= add_tones((alpha_shape >> (r * 8)) & 0xFF, (pattern >> (r * 8)) & 0xFF) << (r * 8);
    return alpha_pattern;
}

static __attribute__((always_inline)) inline u32 add_tones(u32 a, u32 b)
{
    return (((a & 0x55u) + (b & 0x55u)) ^ ((a ^ b) & 0xAAu)) & 0xFFu;
}

static __attribute__((always_inline)) inline v4 five_of(v4 value)
{
    v4 scaled = (value << 5) - value + 127;
    v4 magnitude = (vabs(scaled) * 32897) >> 23;
    return select4((v4)(scaled < vsplat(0)), -magnitude, magnitude);
}

static __attribute__((always_inline)) inline i32 decode_stripes(const Decoding* decoding, i32 target, i32 alpha, u8* restrict out, u8* restrict scratch, i32 first_stripe, i32 stripe_end)
{
    Header header = decoding->header;
    header.has_alpha = alpha;
    const Alphabet* alphabets = decoding->alphabets;
    const u8* tables = (const u8*)alphabets[0].spans;
    const u8* end = decoding->end;
    const u8* data = decoding->stripes;
    i32 stripe_count = decoding->stripe_count;
    if (first_stripe < 0 || first_stripe > stripe_end || stripe_end > stripe_count)
        return -3;

    i32 blocks_wide = header.blocks_wide;
    i32 lane_width = (blocks_wide + LANES - 1) / LANES;
    i32 padded = lane_width * LANES;
    i32* centers = (i32*)(scratch + slot_tables_bytes());
    i32* extents = centers + padded * 3 * 4;
    i32* overrides = extents + padded * 2 * 4;
    i32* alphas = overrides + padded * 2 * 4;
    u32* patterns = (u32*)(alphas + padded * 2);
    Escape* escapes = (Escape*)(((size_t)(patterns + padded * 2) + 15) & ~(size_t)15);
    u8* types = (u8*)(escapes + padded * 2);
    u8* ops = types + padded * 3;
    u8* activities = ops + padded;
    u8* contexts = activities + padded;
    u8* joints = contexts + padded;
    u8* row_tables = joints + padded * 2;
    u8* pair_symbols = row_tables + padded;
    u32* row_fives = (u32*)(((size_t)(pair_symbols + padded) + 3) & ~(size_t)3);
    u32* rows = row_fives + padded;
    u32* alpha_rows = rows + padded;
    u32* alpha_fines = alpha_rows + padded;
    u16* codes = (u16*)(alpha_fines + padded);
    u16* line_list = codes + padded;
    u16* solid_list = line_list + padded;
    u16* table_list = solid_list + padded;
    u16* literal_list = table_list + padded;
    u16* copy_list = literal_list + padded;
    u16* smooth_lists = copy_list + padded;
    i32 clamped_stride = padded + 2;
    u16* payloads = smooth_lists + 2 * padded;
    u16* runs = payloads + 3 * padded;
    v4* clamped = (v4*)(((size_t)(runs + padded) + 15) & ~(size_t)15);
    i32 has_alpha = alpha;
    i32 left_mode = decoding->header.has_alpha | (header.solid_step != 0);
    i32 alpha_step = header.alpha_step;
    v4 default_prediction = { 128, 128, 128, has_alpha ? 128 : 255 };
    i32 lazy_tones = target == TARGET_ASTC_4X4 || target == TARGET_RGBA32 || (!has_alpha && (target == TARGET_BC1 || target == TARGET_BC3 || target == TARGET_ETC2_RGB || target == TARGET_ETC2_RGBA));
    u32 smooth_counts[2] = { 0, 0 };
    v4 solid_steps = vsplat(header.solid_step);
    v4 center_steps[4], extent_steps[4];
    for (i32 activity = 0; activity < 4; activity++)
    {
        i32 step = header.center_steps[activity];
        center_steps[activity] = (v4){ step, step, step, alpha_step };
        extent_steps[activity] = vsplat(header.extent_steps[activity]);
    }
    u32 solid_enabled = header.solid_step != 0;
    v4 opaque_alpha = { 0, 0, 0, has_alpha ? 0 : 255 }, alpha_lanes = { -1, -1, -1, has_alpha ? -1 : 0 };

    for (i32 stripe = 0; stripe < stripe_end; stripe++)
    {
        u32 stripe_bytes = read_u32(decoding->directory + stripe * 4);
        if (stripe_bytes > (u32)(end - data))
            return -3;
        if (stripe < first_stripe)
        {
            data += stripe_bytes;
            continue;
        }
        Plane planes[PLANES * LANES];
        if (!open_planes(data, data + stripe_bytes, planes))
            return -3;
        data += stripe_bytes;
        i32 first_row = stripe * header.stripe_blocks;
        i32 stripe_rows = min32(header.stripe_blocks, header.blocks_high - first_row);
        for (i32 by = 0; by < stripe_rows; by++)
        {
            i32 global_row = first_row + by;
            i32 slot = global_row % 3, up_slot = (global_row + 2) % 3;
            u8* row_types = types + slot * padded;
            const u8* up_types = types + up_slot * padded;
            i32* row_centers = centers + slot * padded * 4;
            const i32* up_centers = centers + up_slot * padded * 4;
            i32 current = by & 1, previous = current ^ 1;
            i32* row_extents = extents + current * padded * 4;
            const i32* up_extents = extents + previous * padded * 4;
            u32* row_patterns = patterns + current * padded;
            const u32* up_patterns = patterns + previous * padded;
            u16* smooth_list = smooth_lists + (global_row & 1) * padded;

            if (by == 0)
                __builtin_memset(contexts, ALPHABET_OPCODE, (size_t)blocks_wide);
            else if (left_mode)
                for (i32 bx = 0; bx < blocks_wide; bx++)
                    contexts[bx] = (u8)(ALPHABET_OPCODE + CONTEXT_CLASS[up_types[bx]]);
            else
                for (i32 bx = 0; bx < blocks_wide; bx++)
                    contexts[bx] = (u8)(ALPHABET_OPCODE + (bx + 1 < blocks_wide ? CONTEXT_CLASS[up_types[bx + 1]] : 0) * 3 + CONTEXT_CLASS[up_types[bx]]);
            if (by == 0)
                __builtin_memset(activities, FIRST_ROW_ACTIVITY, (size_t)blocks_wide);
            else
            {
                const v4* up_clamped = clamped + ((global_row + 2) % 3) * clamped_stride + 1;
                for (i32 bx = 0; bx < blocks_wide; bx++)
                {
                    v4 value = up_clamped[bx];
                    v4 change = vmax(vabs(value - up_clamped[bx - 1]), vabs(value - up_clamped[bx + 1]));
                    i32 spread = max32(max32(change[0], change[1]), change[2]);
                    activities[bx] = (u8)((spread >= 3) + (spread >= 10) + (spread >= 28));
                }
            }

            u32 run_count = 0;
            if (decode_row_symbols(planes, tables, contexts, activities, up_patterns, ops, codes, rows, alpha_rows, alpha_fines, pair_symbols, joints, alphas, overrides, escapes, payloads, runs, &run_count, blocks_wide, lane_width, padded, by, has_alpha, left_mode, end))
                return -6;

            u32 line_count = 0, solid_count = 0, smooth_count = 0, table_count = 0, literal_count = 0, copy_count = 0, invalid = 0;
            if (!left_mode)
            {
                v4 up_center = vload(up_centers), next_up_center = vload(up_centers + min32(1, blocks_wide - 1) * 4), before_up_center = up_center;
                v4 up_extent = vload(up_extents), next_up_extent = vload(up_extents + min32(1, blocks_wide - 1) * 4), before_up_extent = up_extent;
                for (i32 bx = 0; bx < blocks_wide; bx++)
                {
                    u32 op = ops[bx], info = OP_INFO[op], type = info & OP_INFO_TYPE_MASK;
                    invalid |= info >> OP_INFO_INVALID & 1;
                    line_list[line_count] = (u16)bx;
                    line_count += info >> OP_INFO_LINE & 1;
                    smooth_list[smooth_count] = (u16)bx;
                    smooth_count += info >> OP_INFO_SMOOTH & 1;
                    solid_list[solid_count] = (u16)bx;
                    solid_count += info >> OP_INFO_SOLID & 1;
                    table_list[table_count] = (u16)bx;
                    table_count += info >> OP_INFO_TABLED & 1;
                    literal_list[literal_count] = (u16)bx;
                    literal_count += info >> OP_INFO_ROWS & 1;
                    copy_list[copy_count] = (u16)bx;
                    copy_count += info >> OP_INFO_COPY & 1;
                    row_types[bx] = (u8)type;
                    v4 center_residual = residual_of(joints[bx], overrides + bx * 4, alphas[bx], has_alpha);
                    v4 extent_residual = residual_of(joints[padded + bx], overrides + (padded + bx) * 4, alphas[padded + bx], has_alpha);
                    v4 flat = vsplat(0 - (info >> OP_INFO_FLAT & 1)), line = vsplat(0 - (info >> OP_INFO_SPREAD & 1));
                    v4 prediction = by > 0 ? (before_up_center + up_center * 2 + next_up_center + 2) >> 2 : default_prediction;
                    v4 steps = select4(vsplat(0 - ((type == TYPE_SOLID) & solid_enabled)), solid_steps, center_steps[activities[bx]]);
                    v4 center = prediction + center_residual * steps;
                    center = (select4(flat, vclamp255(center), center) & alpha_lanes) | opaque_alpha;
                    vstore(row_centers + bx * 4, center);
                    v4 extent_prediction = by > 0 ? (before_up_extent + up_extent * 2 + next_up_extent + 2) >> 2 : (v4){ 0, 0, 0, 0 };
                    v4 extent = select4(line, extent_prediction + extent_residual * extent_steps[activities[bx]], extent_prediction) & alpha_lanes;
                    vstore(row_extents + bx * 4, extent);
                    row_tables[bx] = 0xFF;
                    row_patterns[bx] = code_pattern(codes[bx]) & (0u - (info >> OP_INFO_CODED & 1));
                    i32 ahead = min32(bx + 2, blocks_wide - 1) * 4;
                    before_up_center = up_center;
                    up_center = next_up_center;
                    next_up_center = vload(up_centers + ahead);
                    before_up_extent = up_extent;
                    up_extent = next_up_extent;
                    next_up_extent = vload(up_extents + ahead);
                }
            }

            for (u32 n = 0; n < table_count; n++)
            {
                i32 bx = table_list[n];
                i32 before = max32(bx - 1, 0), after = min32(bx + 1, blocks_wide - 1);
                v4 center_residual = residual_of(joints[bx], overrides + bx * 4, alphas[bx], has_alpha);
                v4 prediction = by > 0 ? (vload(up_centers + before * 4) + vload(up_centers + bx * 4) * 2 + vload(up_centers + after * 4) + 2) >> 2 : default_prediction;
                v4 green_residual = __builtin_shufflevector(center_residual, center_residual, 1, 1, 1, 1);
                v4 based = five_of(prediction) + green_residual;
                v4 fives = select4((v4){ -1, 0, -1, 0 }, vmin(vmax(based, vsplat(0)), vsplat(31)), based) + ((center_residual - green_residual) & (v4){ -1, 0, -1, 0 });
                fives &= (v4){ -1, -1, -1, 0 };
                v4 outside = (v4)((fives < vsplat(0)) | (fives > vsplat(31)));
                invalid |= (u32)((outside[0] | outside[1] | outside[2]) & 1);
                fives = vmin(vmax(fives, vsplat(0)), vsplat(31));
                v4 center = ((fives << 3) | (fives >> 2)) | (vload(row_centers + bx * 4) & (v4){ 0, 0, 0, -1 });
                vstore(row_centers + bx * 4, center);
                i32 table = joints[padded + bx] & 7, extent = TABLE_EXTENTS[table];
                v4 extent_prediction = by > 0 ? (vload(up_extents + before * 4) + vload(up_extents + bx * 4) * 2 + vload(up_extents + after * 4) + 2) >> 2 : (v4){ 0, 0, 0, 0 };
                vstore(row_extents + bx * 4, (v4){ extent, extent, extent, has_alpha ? extent_prediction[3] + alphas[padded + bx] * extent_steps[activities[bx]][3] : 0 });
                row_tables[bx] = (u8)table;
                row_fives[bx] = (u32)fives[0] | ((u32)fives[1] << 8) | ((u32)fives[2] << 16);
            }
            if (invalid)
                return -7;
            for (u32 n = 0; n < literal_count; n++)
            {
                i32 bx = literal_list[n];
                u32 pattern = literal_pattern(rows[bx], by > 0 ? up_patterns[bx] : 0);
                row_patterns[bx] = pattern;
                if (ops[bx] == OP_DUAL)
                    alpha_rows[bx] = dual_alpha_pattern(alpha_rows[bx], pattern);
            }
            v4 left_center = default_prediction, left_extent = { 0, 0, 0, 0 };
            u32 left_pattern = 0, left_type = TYPE_TRANSPARENT, next_run = 0;
            for (i32 bx = 0; left_mode && bx < blocks_wide; bx++)
            {
                if (next_run < run_count && runs[next_run * 2] == bx)
                {
                    i32 end_x = bx + runs[next_run * 2 + 1];
                    u32 repeated = ops[bx];
                    next_run++;
                    copy_list[copy_count++] = (u16)bx;
                    if (repeated == OP_COPY_LEFT)
                    {
                        if (left_type == TYPE_SMOOTH)
                            return -6;
                        for (i32 x = bx; x < end_x; x++)
                        {
                            row_types[x] = (u8)left_type;
                            row_patterns[x] = left_pattern;
                            vstore(row_centers + x * 4, left_center);
                            vstore(row_extents + x * 4, left_extent);
                        }
                    }
                    else if (repeated == OP_COPY_UP)
                    {
                        for (i32 x = bx; x < end_x; x++)
                        {
                            if (up_types[x] == TYPE_SMOOTH)
                                return -6;
                            row_types[x] = up_types[x];
                            row_patterns[x] = up_patterns[x];
                            vstore(row_centers + x * 4, vload(up_centers + x * 4));
                            vstore(row_extents + x * 4, vload(up_extents + x * 4));
                        }
                    }
                    else if (by > 0)
                        for (i32 x = bx; x < end_x; x++)
                        {
                            row_types[x] = TYPE_TRANSPARENT;
                            row_patterns[x] = 0;
                            vstore(row_centers + x * 4, vload(up_centers + x * 4) & (v4){ -1, -1, -1, 0 });
                            vstore(row_extents + x * 4, vload(up_extents + x * 4));
                        }
                    else
                    {
                        v4 carried = left_center & (v4){ -1, -1, -1, 0 };
                        for (i32 x = bx; x < end_x; x++)
                        {
                            row_types[x] = TYPE_TRANSPARENT;
                            row_patterns[x] = 0;
                            vstore(row_centers + x * 4, carried);
                            vstore(row_extents + x * 4, left_extent);
                        }
                    }
                    bx = end_x - 1;
                    left_center = vload(row_centers + bx * 4);
                    left_extent = vload(row_extents + bx * 4);
                    left_pattern = row_patterns[bx];
                    left_type = row_types[bx];
                    continue;
                }
                u32 op = ops[bx], type, pattern;
                v4 center, extent;
                copy_list[copy_count] = (u16)bx;
                copy_count += (op == OP_COPY_LEFT) | (op == OP_COPY_UP) | (op == OP_TRANSPARENT);
                if (op == OP_COPY_LEFT)
                {
                    if (bx == 0 || left_type == TYPE_SMOOTH)
                        return -6;
                    type = left_type;
                    pattern = left_pattern;
                    center = left_center;
                    extent = left_extent;
                }
                else if (op == OP_COPY_UP)
                {
                    if (by == 0 || up_types[bx] == TYPE_SMOOTH)
                        return -6;
                    type = up_types[bx];
                    pattern = up_patterns[bx];
                    center = vload(up_centers + bx * 4);
                    extent = vload(up_extents + bx * 4);
                }
                else if (op == OP_TRANSPARENT)
                {
                    type = TYPE_TRANSPARENT;
                    pattern = 0;
                    center = (by > 0 ? vload(up_centers + bx * 4) : bx > 0 ? left_center : (v4){ 128, 128, 128, 0 }) & (v4){ -1, -1, -1, 0 };
                    extent = by > 0 ? vload(up_extents + bx * 4) : bx > 0 ? left_extent : (v4){ 0, 0, 0, 0 };
                }
                else
                {
                    if (op >= OP_INVALID)
                        return -7;
                    u32 info = OP_INFO[op];
                    type = info & OP_INFO_TYPE_MASK;
                    pattern = info >> OP_INFO_ROWS & 1 ? literal_pattern(rows[bx], by > 0 ? up_patterns[bx] : 0) : line_pattern(op, codes[bx], rows[bx]);
                    if (op == OP_DUAL)
                        alpha_rows[bx] = dual_alpha_pattern(alpha_rows[bx], pattern);
                    line_list[line_count] = (u16)bx;
                    line_count += info >> OP_INFO_LINE & 1;
                    smooth_list[smooth_count] = (u16)bx;
                    smooth_count += info >> OP_INFO_SMOOTH & 1;
                    solid_list[solid_count] = (u16)bx;
                    solid_count += info >> OP_INFO_SOLID & 1;
                    row_tables[bx] = 0xFF;
                    if (op == OP_PAIR || op == OP_PAIR_LITERAL)
                    {
                        u32 colors[PAIR_SLOTS], count = 0;
                        if (bx > 0)
                            count = add_pair_colors(colors, count, left_type, left_center, left_extent);
                        if (by > 0)
                        {
                            count = add_pair_colors(colors, count, up_types[bx], vload(up_centers + bx * 4), vload(up_extents + bx * 4));
                            if (bx > 0)
                                count = add_pair_colors(colors, count, up_types[bx - 1], vload(up_centers + (bx - 1) * 4), vload(up_extents + (bx - 1) * 4));
                            if (bx + 1 < blocks_wide)
                                count = add_pair_colors(colors, count, up_types[bx + 1], vload(up_centers + (bx + 1) * 4), vload(up_extents + (bx + 1) * 4));
                        }
                        if (has_alpha && count > 0)
                            colors[count++] = 0;
                        u32 pair = pair_symbols[bx];
                        u32 first = (pair - 1) / (PAIR_SLOTS - 1), rest = (pair - 1) % (PAIR_SLOTS - 1);
                        u32 second = rest >= first ? rest + 1 : rest;
                        if (pair == 0 || first >= count || second >= count)
                            return -7;
                        v4 low = unpack_color(colors[first]), high = unpack_color(colors[second]);
                        center = (low + high) >> 1;
                        extent = (high - center) & alpha_lanes;
                        center = (center & alpha_lanes) | opaque_alpha;
                    }
                    else
                    {
                        v4 prediction, extent_prediction;
                        if (bx > 0 && by > 0)
                        {
                            prediction = vmed(left_center, vload(up_centers + bx * 4), vload(up_centers + (bx - 1) * 4));
                            extent_prediction = vmed(left_extent, vload(up_extents + bx * 4), vload(up_extents + (bx - 1) * 4));
                        }
                        else if (bx > 0)
                        {
                            prediction = left_center;
                            extent_prediction = left_extent;
                        }
                        else if (by > 0)
                        {
                            i32 before = max32(bx - 1, 0), after = min32(bx + 1, blocks_wide - 1);
                            prediction = (vload(up_centers + before * 4) + vload(up_centers + bx * 4) * 2 + vload(up_centers + after * 4) + 2) >> 2;
                            extent_prediction = (vload(up_extents + before * 4) + vload(up_extents + bx * 4) * 2 + vload(up_extents + after * 4) + 2) >> 2;
                        }
                        else
                        {
                            prediction = default_prediction;
                            extent_prediction = (v4){ 0, 0, 0, 0 };
                        }
                        v4 center_residual = residual_of(joints[bx], overrides + bx * 4, alphas[bx], has_alpha);
                        v4 extent_residual = residual_of(joints[padded + bx], overrides + (padded + bx) * 4, alphas[padded + bx], has_alpha);
                        v4 flat = vsplat(0 - (info >> OP_INFO_FLAT & 1)), line = vsplat(0 - (info >> OP_INFO_SPREAD & 1));
                        v4 steps = select4(vsplat(0 - ((type == TYPE_SOLID) & solid_enabled)), solid_steps, center_steps[activities[bx]]);
                        center = prediction + center_residual * steps;
                        center = (select4(flat, vclamp255(center), center) & alpha_lanes) | opaque_alpha;
                        extent = select4(line, extent_prediction + extent_residual * extent_steps[activities[bx]], extent_prediction) & alpha_lanes;
                        if (info >> OP_INFO_TABLED & 1)
                        {
                            v4 green_residual = __builtin_shufflevector(center_residual, center_residual, 1, 1, 1, 1);
                            v4 based = five_of(prediction) + green_residual;
                            v4 fives = select4((v4){ -1, 0, -1, 0 }, vmin(vmax(based, vsplat(0)), vsplat(31)), based) + ((center_residual - green_residual) & (v4){ -1, 0, -1, 0 });
                            fives &= (v4){ -1, -1, -1, 0 };
                            v4 outside = (v4)((fives < vsplat(0)) | (fives > vsplat(31)));
                            if ((outside[0] | outside[1] | outside[2]) & 1)
                                return -7;
                            center = ((fives << 3) | (fives >> 2)) | (center & (v4){ 0, 0, 0, -1 });
                            i32 table = joints[padded + bx] & 7, spread = TABLE_EXTENTS[table];
                            extent = (v4){ spread, spread, spread, has_alpha ? extent_prediction[3] + alphas[padded + bx] * extent_steps[activities[bx]][3] : 0 };
                            row_tables[bx] = (u8)table;
                            row_fives[bx] = (u32)fives[0] | ((u32)fives[1] << 8) | ((u32)fives[2] << 16);
                        }
                    }
                }
                row_types[bx] = (u8)type;
                row_patterns[bx] = pattern;
                vstore(row_centers + bx * 4, center);
                vstore(row_extents + bx * 4, extent);
                left_center = center;
                left_extent = extent;
                left_pattern = pattern;
                left_type = type;
            }
            for (u32 n = 0; !left_mode && n < copy_count; n++)
            {
                i32 bx = copy_list[n];
                i32 left = ops[bx] == OP_COPY_LEFT;
                u32 source_type = left ? row_types[bx > 0 ? bx - 1 : 0] : up_types[bx];
                invalid |= left ? bx == 0 : by == 0;
                invalid |= source_type == TYPE_SMOOTH;
                if (invalid)
                    return -6;
                row_types[bx] = (u8)source_type;
                const i32* source_centers = left ? row_centers + (bx - 1) * 4 : up_centers + bx * 4;
                const i32* source_extents = left ? row_extents + (bx - 1) * 4 : up_extents + bx * 4;
                vstore(row_centers + bx * 4, vload(source_centers));
                vstore(row_extents + bx * 4, vload(source_extents));
                row_patterns[bx] = left ? row_patterns[bx - 1] : up_patterns[bx];
            }

            if (target == TARGET_BC1 && !has_alpha && !header.mip_side)
                emit_bc1_lines((u64*)out + (size_t)global_row * blocks_wide, line_list, line_count, row_centers, row_extents, row_patterns);
            else if (target == TARGET_BC3 && has_alpha && !header.mip_side)
                emit_bc3_lines((u64*)out + (size_t)global_row * blocks_wide * 2, line_list, line_count, row_centers, row_extents, row_patterns, ops, alpha_rows, alpha_fines);
            else if ((target == TARGET_BC7 || target == TARGET_ASTC_4X4) && !has_alpha && !header.mip_side)
                emit_wide_lines_opaque((u64*)out + (size_t)global_row * blocks_wide * 2, target, line_list, line_count, row_centers, row_extents, row_patterns);
            else if ((target == TARGET_ETC2_RGB || target == TARGET_ETC2_RGBA) && !has_alpha && !header.mip_side && !header.etc_split)
                emit_etc_lines_opaque(out + (size_t)global_row * blocks_wide * (target == TARGET_ETC2_RGBA ? 16 : 8), target, line_list, line_count, row_centers, row_extents, row_patterns, row_tables, row_fives);
            else
                for (u32 n = 0; n < line_count; n++)
                {
                    i32 bx = line_list[n];
                    v4 center = vload(row_centers + bx * 4), extent = vload(row_extents + bx * 4);
                    Block block;
                    block.transparent = 0;
                    block.field = 0;
                    block.table = row_tables[bx] == 0xFF ? -1 : row_tables[bx];
                    block.base5[0] = (i32)(row_fives[bx] & 0xFF);
                    block.base5[1] = (i32)((row_fives[bx] >> 8) & 0xFF);
                    block.base5[2] = (i32)(row_fives[bx] >> 16);
                    block.dual = ops[bx] == OP_DUAL;
                    block.tones = row_patterns[bx];
                    block.alpha_tones = block.dual ? alpha_rows[bx] : row_patterns[bx];
                    block.alpha_fine = block.dual ? alpha_fines[bx] : 0;
                    vbytes(block.low, vclamp255(center - extent));
                    vbytes(block.high, vclamp255(center + extent));
                    vbytes(block.center, vclamp255(center));
                    emit_block(out, target, &header, bx, global_row, &block);
                }
            const u8* repeated = 0;
            u32 repeated_color = 0;
            for (u32 n = 0; n < solid_count; n++)
                {
                    i32 bx = solid_list[n];
                    emit_flat_solid_block(out, target, &header, pack_color(vload(row_centers + bx * 4)), global_row, bx, &repeated, &repeated_color);
                }
            i32 bulk = target != TARGET_RGBA32 && !header.mip_side, block_bytes = BYTES_PER_BLOCK[target];
            u8* row_blocks = out + (size_t)global_row * blocks_wide * block_bytes;
            for (u32 n = 0, next_run = 0; n < copy_count; n++)
            {
                i32 bx = copy_list[n], end_x = bx + 1;
                u32 op = ops[bx];
                if (next_run < run_count && runs[next_run * 2] == bx)
                    end_x = bx + runs[next_run++ * 2 + 1];
                if (bulk && end_x - bx > 1)
                {
                    if (op == OP_COPY_UP)
                        __builtin_memcpy(row_blocks + (size_t)bx * block_bytes, row_blocks - (size_t)blocks_wide * block_bytes + (size_t)bx * block_bytes, (size_t)(end_x - bx) * block_bytes);
                    else
                    {
                        const u8* source = op == OP_COPY_LEFT ? row_blocks + (size_t)(bx - 1) * block_bytes : TRANSPARENT_BLOCKS[target];
                        for (i32 x = bx; x < end_x; x++)
                            copy_block_bytes(row_blocks + (size_t)x * block_bytes, source, block_bytes);
                    }
                }
                else
                    for (i32 x = bx; x < end_x; x++)
                        if (op == OP_TRANSPARENT)
                            emit_transparent(out, target, &header, x, global_row);
                        else
                            copy_emitted_block(out, target, &header, x, global_row, op == OP_COPY_LEFT ? x - 1 : x, op == OP_COPY_LEFT ? global_row : global_row - 1);
                while (n + 1 < copy_count && copy_list[n + 1] < end_x)
                    n++;
            }
            smooth_counts[global_row & 1] = smooth_count;
            build_clamped_row(clamped + (global_row % 3) * clamped_stride, row_centers, blocks_wide);
            if (by > 0)
            {
                i32 row = global_row - 1, has_up = by > 1;
                emit_smooth_row(out, target, &header, row, smooth_lists + (row & 1) * padded, smooth_counts[row & 1],
                    has_up ? centers + ((row + 2) % 3) * padded * 4 : 0, types + ((row + 2) % 3) * padded,
                    centers + (row % 3) * padded * 4, types + (row % 3) * padded, row_centers, row_types, lazy_tones,
                    clamped + ((has_up ? row + 2 : row) % 3) * clamped_stride, clamped + (row % 3) * clamped_stride, clamped + (global_row % 3) * clamped_stride);
            }
            if (by == stripe_rows - 1)
            {
                i32 has_up = by > 0;
                emit_smooth_row(out, target, &header, global_row, smooth_list, smooth_count, has_up ? up_centers : 0, up_types, row_centers, row_types, 0, 0, lazy_tones,
                    clamped + ((has_up ? global_row + 2 : global_row) % 3) * clamped_stride, clamped + (global_row % 3) * clamped_stride, clamped + (global_row % 3) * clamped_stride);
            }
        }
    }
    return 1;
}

#ifdef __wasm__
static i32 decode_wasm_stripes(i32 target, u8* out, i32 first_stripe, i32 stripe_end);
#endif

static __attribute__((noinline)) i32 decode_rgba_stripes(const Decoding* decoding, u8* restrict out, u8* restrict scratch, i32 first_stripe, i32 stripe_end)
{
#ifdef __wasm__
    (void)decoding;
    (void)scratch;
    return decode_wasm_stripes(TARGET_RGBA32, out, first_stripe, stripe_end);
#else
    if (decoding->header.has_alpha)
        return decode_stripes(decoding, TARGET_RGBA32, 1, out, scratch, first_stripe, stripe_end);
    return decode_stripes(decoding, TARGET_RGBA32, 0, out, scratch, first_stripe, stripe_end);
#endif
}

#ifndef __wasm__
static __attribute__((always_inline)) inline i32 decode(const u8* restrict file, i32 length, i32 target, u8* restrict out, u8* restrict scratch, Decoding* decoding, i32 inherit, i32 etc_split, u32* side, i32 first_stripe, i32 stripe_end)
{
    i32 status = begin_decoding(file, length, scratch, decoding, inherit);
    if (status != 1)
        return status;
    decoding->header.etc_split = etc_split;
    decoding->header.mip_side = side;
    decoding->header.side_width = max32(1, decoding->header.width >> 1);
    decoding->header.side_height = max32(1, decoding->header.height >> 1);
    stripe_end = min32(stripe_end, decoding->stripe_count);
    if (target == TARGET_RGBA32)
        return decode_rgba_stripes(decoding, out, scratch, first_stripe, stripe_end);
    if (decoding->header.has_alpha)
        return decode_stripes(decoding, target, 1, out, scratch, first_stripe, stripe_end);
    return decode_stripes(decoding, target, 0, out, scratch, first_stripe, stripe_end);
}
#endif

static i32 large_block_of(i32 target) { return target == TARGET_ASTC_6X6 ? 6 : target == TARGET_ASTC_8X8 ? 8 : 0; }

static const AstcLargeEncoder* large_encoder_of(i32 target) { return target == TARGET_ASTC_6X6 ? &ASTC_6X6_ENCODER : &ASTC_8X8_ENCODER; }

static i32 greatest_divisor(i32 a, i32 b)
{
    while (b)
    {
        i32 rest = a % b;
        a = b;
        b = rest;
    }
    return a;
}

static i32 large_stripe_group(const Header* header, i32 block) { return block / greatest_divisor(header->stripe_blocks * 4, block); }

static size_t large_rows_bytes(const Header* header, i32 block) { return ((size_t)header->width * (size_t)(header->stripe_blocks * 4 + block) * 4 + 15) & ~(size_t)15; }

static i32 decode_large_stripes(Decoding* decoding, i32 target, u8* restrict out, u8* restrict scratch, u32* restrict rows, i32 first_stripe, i32 stripe_end)
{
    Header* header = &decoding->header;
    const AstcLargeEncoder* encoder = large_encoder_of(target);
    i32 block = encoder->block, stripe_pixels = header->stripe_blocks * 4, blocks_wide = (header->width + block - 1) / block, carried = 0;
    i32 group = large_stripe_group(header, block);
    stripe_end = min32(stripe_end, decoding->stripe_count);
    if (first_stripe % group != 0 || (stripe_end % group != 0 && stripe_end != decoding->stripe_count))
        return -3;
    for (i32 stripe = first_stripe; stripe < stripe_end; stripe++)
    {
        i32 top = stripe * stripe_pixels, bottom = min32(top + stripe_pixels, header->height);
        header->rgba_row_base = top - carried;
        i32 status = decode_rgba_stripes(decoding, (u8*)rows, scratch, stripe, stripe + 1);
        if (status != 1)
            return status;
        i32 buffered = carried + bottom - top, encoded = bottom == header->height ? buffered : buffered / block * block;
        encode_large_rows(rows, header->width, encoded, buffered, encoder, out + (size_t)((top - carried) / block) * blocks_wide * 16);
        carried = buffered - encoded;
        __builtin_memmove(rows, rows + (size_t)encoded * header->width, (size_t)carried * header->width * 4);
    }
    header->rgba_row_base = 0;
    return 1;
}

#ifndef __wasm__
static i32 decode_large(const u8* file, i32 length, i32 target, u8* restrict out, u8* restrict scratch, u32* restrict rows, Decoding* decoding, i32 inherit, u32* side, i32 first_stripe, i32 stripe_end)
{
    i32 status = begin_decoding(file, length, scratch, decoding, inherit);
    if (status != 1)
        return status;
    decoding->header.mip_side = side;
    decoding->header.side_width = max32(1, decoding->header.width >> 1);
    decoding->header.side_height = max32(1, decoding->header.height >> 1);
    return decode_large_stripes(decoding, target, out, scratch, rows, first_stripe, stripe_end);
}
#endif

static u64 level_bytes(i32 width, i32 height, i32 target)
{
    i32 block = large_block_of(target);
    if (block)
        return (u64)((width + block - 1) / block) * (u64)((height + block - 1) / block) * 16;
    return target == TARGET_RGBA32 ? (u64)width * height * 4 : (u64)((width + 3) >> 2) * ((height + 3) >> 2) * BYTES_PER_BLOCK[target];
}

static u64 mip_work_bytes(i32 width, i32 height)
{
    i32 side_width = max32(1, width >> 1), side_height = max32(1, height >> 1);
    return ((u64)side_width * side_height + (u64)max32(1, side_width >> 1) * max32(1, side_height >> 1)) * 4 + 16;
}

static inline i32 hmin(v4 value) { return min32(min32(value[0], value[1]), min32(value[2], value[3])); }

static inline i32 hmax(v4 value) { return max32(max32(value[0], value[1]), max32(value[2], value[3])); }

static void fit_line_block(const u32 pixels[16], i32 has_alpha, Block* block)
{
    __builtin_memset(block, 0, sizeof(Block));
    block->table = -1;
    v4 channels[4][4];
    v4 minimum = vsplat(255), maximum = vsplat(0), sum = vsplat(0);
    for (i32 g = 0; g < 4; g++)
    {
        v4 group = vload((const i32*)pixels + g * 4);
        for (i32 c = 0; c < 4; c++)
            channels[g][c] = (group >> (c * 8)) & 255;
    }
    i32 low_values[4], high_values[4], sums[4];
    for (i32 c = 0; c < 4; c++)
    {
        v4 channel_min = vmin(vmin(channels[0][c], channels[1][c]), vmin(channels[2][c], channels[3][c]));
        v4 channel_max = vmax(vmax(channels[0][c], channels[1][c]), vmax(channels[2][c], channels[3][c]));
        low_values[c] = hmin(channel_min);
        high_values[c] = hmax(channel_max);
        sums[c] = vsum(channels[0][c] + channels[1][c] + channels[2][c] + channels[3][c]);
    }
    minimum = vload(low_values);
    maximum = vload(high_values);
    sum = vload(sums);
    if (has_alpha && maximum[3] == 0)
    {
        block->transparent = 1;
        return;
    }
    v4 lanes = { 1, 1, 1, has_alpha };
    v4 range = (maximum - minimum) * lanes;
    i32 reference = 0;
    for (i32 c = 1; c < 4; c++)
        if (range[c] > range[reference])
            reference = c;
    v4 low = minimum, high = maximum;
    if (range[reference] > 0)
    {
        v4 mean = (sum + 8) >> 4;
        i32 covariance[4];
        for (i32 c = 0; c < 4; c++)
        {
            v4 total = vsplat(0);
            for (i32 g = 0; g < 4; g++)
                total += (channels[g][c] - mean[c]) * (channels[g][reference] - mean[reference]);
            covariance[c] = vsum(total);
        }
        v4 swap = vload(covariance) < 0;
        v4 first = (swap & maximum) | (~swap & minimum), second = (swap & minimum) | (~swap & maximum);
        v4 inset = (second - first) >> 4;
        low = first + inset;
        high = second - inset;
    }
    if (!has_alpha)
    {
        low[3] = 255;
        high[3] = 255;
    }
    v4 direction = (high - low) * lanes;
    i32 norm = vsum(direction * direction);
    u32 tones = 0;
    if (norm > 0)
    {
        v4 first = vsplat(norm), second = vsplat(norm * 3), third = vsplat(norm * 5);
        const v4 weights = { 1, 4, 16, 64 };
        for (i32 g = 0; g < 4; g++)
        {
            v4 along = vsplat(0);
            for (i32 c = 0; c < 4; c++)
                along += (channels[g][c] - low[c]) * direction[c];
            along *= 6;
            v4 tone = -((v4)(along >= first) + (v4)(along >= second) + (v4)(along >= third));
            tones |= (u32)vsum(tone * weights) << (g * 8);
        }
    }
    vbytes(block->low, low);
    vbytes(block->high, high);
    vbytes(block->center, (low + high) >> 1);
    block->tones = tones;
}

static void emit_level(const u32* rgba, i32 width, i32 height, i32 target, i32 has_alpha, u8* out)
{
    if (large_block_of(target))
    {
        encode_large_rows(rgba, width, height, height, large_encoder_of(target), out);
        return;
    }
    if (target == TARGET_RGBA32)
    {
        __builtin_memcpy(out, rgba, (size_t)width * height * 4);
        return;
    }
    i32 blocks_wide = (width + 3) >> 2, blocks_high = (height + 3) >> 2;
    for (i32 by = 0; by < blocks_high; by++)
        for (i32 bx = 0; bx < blocks_wide; bx++)
        {
            u32 pixels[16];
            if (bx * 4 + 4 <= width && by * 4 + 4 <= height)
                for (i32 y = 0; y < 4; y++)
                    __builtin_memcpy(pixels + y * 4, rgba + (size_t)(by * 4 + y) * width + bx * 4, 16);
            else
                for (i32 y = 0; y < 4; y++)
                    for (i32 x = 0; x < 4; x++)
                        pixels[y * 4 + x] = rgba[(size_t)min32(by * 4 + y, height - 1) * width + min32(bx * 4 + x, width - 1)];
            Block block;
            fit_line_block(pixels, has_alpha, &block);
            u8* at = out + ((size_t)by * blocks_wide + bx) * BYTES_PER_BLOCK[target];
            if (block.transparent)
                __builtin_memcpy(at, TRANSPARENT_BLOCKS[target], (size_t)BYTES_PER_BLOCK[target]);
            else
                emit_block_bytes(at, target, has_alpha, 0, &block);
        }
}

static void half_level(const u32* source, i32 width, i32 height, u32* destination)
{
    i32 half_width = max32(1, width >> 1), half_height = max32(1, height >> 1);
    for (i32 y = 0; y < half_height; y++)
        for (i32 x = 0; x < half_width; x++)
        {
            i32 x0 = min32(x * 2, width - 1), x1 = min32(x * 2 + 1, width - 1), y0 = min32(y * 2, height - 1), y1 = min32(y * 2 + 1, height - 1);
            v4 total = unpack_color(source[(size_t)y0 * width + x0]) + unpack_color(source[(size_t)y0 * width + x1]) + unpack_color(source[(size_t)y1 * width + x0]) + unpack_color(source[(size_t)y1 * width + x1]);
            destination[(size_t)y * half_width + x] = pack_color((total + 2) >> 2);
        }
}

static void build_lower_levels(u32* side, i32 width, i32 height, i32 target, i32 has_alpha, u8* out)
{
    i32 level_width = max32(1, width >> 1), level_height = max32(1, height >> 1);
    u32* source = side;
    u32* spare = side + (size_t)level_width * level_height;
    if (width == 1 && height == 1)
        return;
    for (;;)
    {
        emit_level(source, level_width, level_height, target, has_alpha, out);
        out += level_bytes(level_width, level_height, target);
        if (level_width == 1 && level_height == 1)
            return;
        half_level(source, level_width, level_height, spare);
        u32* swap = source;
        source = spare;
        spare = swap;
        level_width = max32(1, level_width >> 1);
        level_height = max32(1, level_height >> 1);
    }
}

#ifndef __wasm__
#if defined(_WIN32) && defined(QTX_WITHOUT_LIBC)
int _fltused = 0;
#endif

enum { MIP_LEVELS_MAX = QTX_MAX_LEVELS, MIP_HEADER_BYTES = 8 };

static i32 mip_level_count(const u8* file, i32 length)
{
    if (length < MIP_HEADER_BYTES || file[0] != 'Q' || file[1] != 'T' || file[2] != 'X' || file[3] != 'M')
        return 0;
    i32 count = file[4];
    return count >= 1 && count <= MIP_LEVELS_MAX && length >= MIP_HEADER_BYTES + count * 4 ? count : 0;
}

static i32 split_levels(const u8* file, i32 length, const u8** files, i32* lengths, Header* headers)
{
    i32 count = mip_level_count(file, length);
    if (!count)
    {
        files[0] = file;
        lengths[0] = length;
        return read_header(file, length, &headers[0]);
    }
    u32 offset = MIP_HEADER_BYTES + (u32)count * 4;
    for (i32 level = 0; level < count; level++)
    {
        const u8* size_bytes = file + MIP_HEADER_BYTES + level * 4;
        u32 size = size_bytes[0] | (size_bytes[1] << 8) | (size_bytes[2] << 16) | ((u32)size_bytes[3] << 24);
        if (size > (u32)length - offset)
            return 0;
        files[level] = file + offset;
        lengths[level] = (i32)size;
        if (!read_header(files[level], lengths[level], &headers[level]))
            return 0;
        offset += size;
    }
    return count;
}

static i32 decode_target(const u8* file, i32 length, i32 target, u8* restrict out, u8* restrict scratch, Decoding* decoding, i32 inherit, u32* side, i32 first_stripe, i32 stripe_end)
{
    switch (target)
    {
        case 0: return decode(file, length, 0, out, scratch, decoding, inherit, 0, side, first_stripe, stripe_end);
        case 1: return decode(file, length, 1, out, scratch, decoding, inherit, 0, side, first_stripe, stripe_end);
        case 2: return decode(file, length, 2, out, scratch, decoding, inherit, 0, side, first_stripe, stripe_end);
        case 3: return decode(file, length, 3, out, scratch, decoding, inherit, 0, side, first_stripe, stripe_end);
        case 4: return decode(file, length, 4, out, scratch, decoding, inherit, 0, side, first_stripe, stripe_end);
        case 5: return decode(file, length, 5, out, scratch, decoding, inherit, 0, side, first_stripe, stripe_end);
        case 6: return decode(file, length, 6, out, scratch, decoding, inherit, 0, side, first_stripe, stripe_end);
        case TARGET_ETC2_RGB_SPLIT: return decode(file, length, TARGET_ETC2_RGB, out, scratch, decoding, inherit, 1, side, first_stripe, stripe_end);
        case TARGET_ETC2_RGBA_SPLIT: return decode(file, length, TARGET_ETC2_RGBA, out, scratch, decoding, inherit, 1, side, first_stripe, stripe_end);
        default: return 0;
    }
}

typedef struct
{
    const u8* files[MIP_LEVELS_MAX];
    i32 lengths[MIP_LEVELS_MAX];
    Header headers[MIP_LEVELS_MAX];
    i32 stored_levels;
    i32 target;
    i32 build_mips;
    u32 level_count;
    u32 level_width[MIP_LEVELS_MAX];
    u32 level_height[MIP_LEVELS_MAX];
    size_t level_offset[MIP_LEVELS_MAX];
    size_t level_size[MIP_LEVELS_MAX];
    size_t output_size;
    size_t scratch_size;
    u32 work_units;
} Layout;

static i32 is_level_magic(const u8* file, i32 length)
{
    return length >= 4 && file[0] == 'Q' && file[1] == 'T' && file[2] == 'X' && file[3] == ' ';
}

static qtx_result failure_of(const u8* file, i32 length)
{
    const u8* level = file;
    i32 level_length = length;
    i32 container = length >= 4 && file[0] == 'Q' && file[1] == 'T' && file[2] == 'X' && file[3] == 'M';
    if (container)
    {
        i32 count = length >= MIP_HEADER_BYTES ? file[4] : 0;
        if (count < 1 || count > MIP_LEVELS_MAX || length < MIP_HEADER_BYTES + count * 4)
            return QTX_ERROR_CORRUPT_DATA;
        level = file + MIP_HEADER_BYTES + count * 4;
        level_length = length - (MIP_HEADER_BYTES + count * 4);
    }
    if (!is_level_magic(level, level_length))
        return container ? QTX_ERROR_CORRUPT_DATA : QTX_ERROR_NOT_QTX;
    if (level_length >= HEADER_BYTES && level[8] != FORMAT_VERSION)
        return QTX_ERROR_UNSUPPORTED_VERSION;
    return QTX_ERROR_CORRUPT_DATA;
}

static qtx_result read_levels(const void* data, size_t size, Layout* layout)
{
    if (!data || size > 0x7FFFFFFF)
        return QTX_ERROR_INVALID_ARGUMENT;
    layout->stored_levels = split_levels(data, (i32)size, layout->files, layout->lengths, layout->headers);
    return layout->stored_levels ? QTX_SUCCESS : failure_of(data, (i32)size);
}

static i32 add_level(Layout* layout, i32 width, i32 height, i32 base)
{
    u64 bytes = level_bytes(width, height, base);
    if (bytes > (size_t)-1 - layout->output_size)
        return 0;
    u32 level = layout->level_count++;
    layout->level_width[level] = (u32)width;
    layout->level_height[level] = (u32)height;
    layout->level_offset[level] = layout->output_size;
    layout->level_size[level] = (size_t)bytes;
    layout->output_size += layout->level_size[level];
    return 1;
}

static u32 level_stripes(const Header* header) { return (u32)((header->blocks_high + header->stripe_blocks - 1) / header->stripe_blocks); }

static u32 level_units(const Header* header, i32 target)
{
    i32 block = large_block_of(target);
    if (!block)
        return level_stripes(header);
    u32 group = (u32)large_stripe_group(header, block);
    return (level_stripes(header) + group - 1) / group;
}

static size_t decode_scratch_bytes(const Header* header, i32 target)
{
    size_t bytes = (scratch_bytes(header) + 15) & ~(size_t)15;
    i32 block = large_block_of(target);
    return block ? bytes + large_rows_bytes(header, block) : bytes;
}

static qtx_result read_layout(const void* data, size_t size, qtx_format format, uint32_t flags, Layout* layout)
{
    if ((u32)format > QTX_FORMAT_ASTC_8x8_RGBA)
        return QTX_ERROR_UNSUPPORTED_FORMAT;
    if (flags & ~(uint32_t)(QTX_TRANSCODE_BUILD_MIPS | QTX_TRANSCODE_HIGH_QUALITY))
        return QTX_ERROR_INVALID_ARGUMENT;
    qtx_result result = read_levels(data, size, layout);
    if (result != QTX_SUCCESS)
        return result;
    i32 high_quality = (flags & QTX_TRANSCODE_HIGH_QUALITY) && (format == QTX_FORMAT_ETC2_RGB || format == QTX_FORMAT_ETC2_RGBA);
    layout->target = high_quality ? (format == QTX_FORMAT_ETC2_RGB ? TARGET_ETC2_RGB_SPLIT : TARGET_ETC2_RGBA_SPLIT) : (i32)format;
    layout->build_mips = (flags & QTX_TRANSCODE_BUILD_MIPS) && layout->stored_levels == 1;
    layout->level_count = 0;
    layout->output_size = 0;
    layout->scratch_size = 0;
    if (layout->build_mips)
    {
        const Header* header = &layout->headers[0];
        i32 width = header->width, height = header->height;
        if (!add_level(layout, width, height, (i32)format))
            return QTX_ERROR_INVALID_ARGUMENT;
        while (width > 1 || height > 1)
        {
            width = max32(1, width >> 1);
            height = max32(1, height >> 1);
            if (!add_level(layout, width, height, (i32)format))
                return QTX_ERROR_INVALID_ARGUMENT;
        }
        layout->scratch_size = decode_scratch_bytes(header, layout->target);
        u64 work = mip_work_bytes(header->width, header->height);
        if (work > (size_t)-1 - layout->scratch_size)
            return QTX_ERROR_INVALID_ARGUMENT;
        layout->scratch_size += (size_t)work;
        layout->work_units = 1;
        return QTX_SUCCESS;
    }
    layout->work_units = 0;
    for (i32 level = 0; level < layout->stored_levels; level++)
    {
        const Header* header = &layout->headers[level];
        layout->work_units += level_units(header, layout->target);
        if (!add_level(layout, layout->headers[level].width, layout->headers[level].height, (i32)format))
            return QTX_ERROR_INVALID_ARGUMENT;
        if (decode_scratch_bytes(header, layout->target) > layout->scratch_size)
            layout->scratch_size = decode_scratch_bytes(header, layout->target);
    }
    return QTX_SUCCESS;
}

QTX_API void qtx_init(void)
{
    if (!initialized)
        initialize();
}

QTX_API uint32_t qtx_version(void)
{
    return (QTX_VERSION_MAJOR << 16) | (QTX_VERSION_MINOR << 8) | QTX_VERSION_PATCH;
}

QTX_API const char* qtx_result_string(qtx_result result)
{
    switch (result)
    {
        case QTX_SUCCESS: return "success";
        case QTX_ERROR_INVALID_ARGUMENT: return "invalid argument";
        case QTX_ERROR_NOT_QTX: return "the data is not a QTX image";
        case QTX_ERROR_UNSUPPORTED_VERSION: return "the QTX format version is not supported by this decoder";
        case QTX_ERROR_CORRUPT_DATA: return "the QTX data is corrupt or truncated";
        case QTX_ERROR_BUFFER_TOO_SMALL: return "the output or scratch buffer is too small";
        case QTX_ERROR_UNSUPPORTED_FORMAT: return "the target format is not supported";
        case QTX_ERROR_INTERNAL: return "internal error";
        default: return "unknown result";
    }
}

QTX_API qtx_result qtx_get_image_info(const void* data, size_t size, qtx_image_info* info)
{
    Layout layout;
    if (!info)
        return QTX_ERROR_INVALID_ARGUMENT;
    qtx_result result = read_levels(data, size, &layout);
    if (result != QTX_SUCCESS)
        return result;
    info->width = (u32)layout.headers[0].width;
    info->height = (u32)layout.headers[0].height;
    info->level_count = (u32)layout.stored_levels;
    info->has_alpha = (u32)layout.headers[0].has_alpha;
    info->format_version = FORMAT_VERSION;
    return QTX_SUCCESS;
}

QTX_API qtx_result qtx_get_transcode_info(const void* data, size_t size, qtx_format format, uint32_t flags, qtx_transcode_info* info)
{
    Layout layout;
    if (!info)
        return QTX_ERROR_INVALID_ARGUMENT;
    qtx_result result = read_layout(data, size, format, flags, &layout);
    if (result != QTX_SUCCESS)
        return result;
    info->output_size = layout.output_size;
    info->scratch_size = layout.scratch_size;
    info->level_count = layout.level_count;
    info->work_units = layout.work_units;
    return QTX_SUCCESS;
}

QTX_API qtx_result qtx_get_level_info(const void* data, size_t size, qtx_format format, uint32_t flags, uint32_t level, qtx_level_info* info)
{
    Layout layout;
    if (!info)
        return QTX_ERROR_INVALID_ARGUMENT;
    qtx_result result = read_layout(data, size, format, flags, &layout);
    if (result != QTX_SUCCESS)
        return result;
    if (level >= layout.level_count)
        return QTX_ERROR_INVALID_ARGUMENT;
    info->width = layout.level_width[level];
    info->height = layout.level_height[level];
    info->offset = layout.level_offset[level];
    info->size = layout.level_size[level];
    return QTX_SUCCESS;
}

QTX_API qtx_result qtx_transcode_units(const void* data, size_t size, qtx_format format, uint32_t flags, uint32_t first_unit, uint32_t unit_count, void* output, size_t output_size, void* scratch, size_t scratch_size)
{
    Layout layout;
    qtx_result result = read_layout(data, size, format, flags, &layout);
    if (result != QTX_SUCCESS)
        return result;
    if (!output || !scratch || first_unit > layout.work_units || unit_count > layout.work_units - first_unit)
        return QTX_ERROR_INVALID_ARGUMENT;
    if (output_size < layout.output_size || scratch_size < layout.scratch_size)
        return QTX_ERROR_BUFFER_TOO_SMALL;
    qtx_init();
    u8* out = output;
    Decoding decoding;
    if (layout.build_mips)
    {
        if (unit_count == 0)
            return QTX_SUCCESS;
        const Header* header = &layout.headers[0];
        u32* rows = (u32*)((u8*)scratch + ((scratch_bytes(header) + 15) & ~(size_t)15));
        u32* side = (u32*)((u8*)scratch + decode_scratch_bytes(header, layout.target));
        i32 status = large_block_of(layout.target)
            ? decode_large(layout.files[0], layout.lengths[0], layout.target, out, scratch, rows, &decoding, 0, side, 0, 1 << 30)
            : decode_target(layout.files[0], layout.lengths[0], layout.target, out, scratch, &decoding, 0, side, 0, 1 << 30);
        if (status != 1)
            return QTX_ERROR_CORRUPT_DATA;
        build_lower_levels(side, header->width, header->height, (i32)format, header->has_alpha, out + layout.level_size[0]);
        return QTX_SUCCESS;
    }
    u32 end_unit = first_unit + unit_count, level_start = 0;
    i32 tables_loaded = 0, block = large_block_of(layout.target);
    for (i32 level = 0; level < layout.stored_levels; level++)
    {
        const Header* header = &layout.headers[level];
        u32 units = level_units(header, layout.target), stripes = level_stripes(header);
        u32 first = first_unit > level_start ? first_unit - level_start : 0, end = end_unit - level_start < units ? end_unit - level_start : units;
        if (end_unit > level_start && first < end)
        {
            if (level > 0 && header->shared_tables && !tables_loaded && begin_decoding(layout.files[0], layout.lengths[0], scratch, &decoding, 0) != 1)
                return QTX_ERROR_CORRUPT_DATA;
            u8* level_out = out + layout.level_offset[level];
            i32 status;
            if (block)
            {
                u32 group = (u32)large_stripe_group(header, block);
                u32* rows = (u32*)((u8*)scratch + ((scratch_bytes(header) + 15) & ~(size_t)15));
                status = decode_large(layout.files[level], layout.lengths[level], layout.target, level_out, scratch, rows, &decoding, level > 0, 0, (i32)(first * group), (i32)(end * group < stripes ? end * group : stripes));
            }
            else
                status = decode_target(layout.files[level], layout.lengths[level], layout.target, level_out, scratch, &decoding, level > 0, 0, (i32)first, (i32)end);
            if (status != 1)
                return QTX_ERROR_CORRUPT_DATA;
            tables_loaded = 1;
        }
        level_start += units;
    }
    return QTX_SUCCESS;
}

QTX_API qtx_result qtx_transcode(const void* data, size_t size, qtx_format format, uint32_t flags, void* output, size_t output_size, void* scratch, size_t scratch_size)
{
    qtx_transcode_info info;
    qtx_result result = qtx_get_transcode_info(data, size, format, flags, &info);
    if (result != QTX_SUCCESS)
        return result;
    return qtx_transcode_units(data, size, format, flags, 0, info.work_units, output, output_size, scratch, scratch_size);
}
#endif

#ifdef __wasm__
#include "wasm_api.c"
#endif
