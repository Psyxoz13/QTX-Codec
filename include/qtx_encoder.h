#ifndef QTX_ENCODER_H
#define QTX_ENCODER_H

#include "qtx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define QTX_ENCODE_DEFAULT_QUALITY 50

typedef enum qtx_encode_flags
{
    QTX_ENCODE_DEFAULT = 0,
    QTX_ENCODE_MIPS = 1
} qtx_encode_flags;

typedef struct qtx_encode_params
{
    uint32_t struct_size;
    int32_t quality;
    uint32_t flags;
} qtx_encode_params;

#define QTX_ENCODE_PARAMS_INIT { sizeof(qtx_encode_params), QTX_ENCODE_DEFAULT_QUALITY, QTX_ENCODE_DEFAULT }

qtx_result qtx_encode(const void* rgba, uint32_t width, uint32_t height, const qtx_encode_params* params, void** output, size_t* output_size);
qtx_result qtx_encode_levels(const void* levels, uint32_t level_count, uint32_t width, uint32_t height, const qtx_encode_params* params, void** output, size_t* output_size);
void qtx_free(void* memory);

#ifdef __cplusplus
}
#endif

#endif
