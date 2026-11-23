#ifndef QTX_H
#define QTX_H

#include <stddef.h>
#include <stdint.h>

#if defined(QTX_SHARED) && defined(QTX_BUILD) && defined(_WIN32)
#define QTX_API __declspec(dllexport)
#elif defined(QTX_SHARED) && defined(_WIN32)
#define QTX_API __declspec(dllimport)
#elif defined(QTX_SHARED) && defined(QTX_BUILD)
#define QTX_API __attribute__((visibility("default")))
#else
#define QTX_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define QTX_VERSION_MAJOR 1
#define QTX_VERSION_MINOR 0
#define QTX_VERSION_PATCH 0
#define QTX_MAX_LEVELS 16

typedef enum qtx_result
{
    QTX_SUCCESS = 0,
    QTX_ERROR_INVALID_ARGUMENT = -1,
    QTX_ERROR_NOT_QTX = -2,
    QTX_ERROR_UNSUPPORTED_VERSION = -3,
    QTX_ERROR_CORRUPT_DATA = -4,
    QTX_ERROR_BUFFER_TOO_SMALL = -5,
    QTX_ERROR_UNSUPPORTED_FORMAT = -6,
    QTX_ERROR_INTERNAL = -7
} qtx_result;

typedef enum qtx_format
{
    QTX_FORMAT_RGBA8 = 0,
    QTX_FORMAT_BC1_RGB = 1,
    QTX_FORMAT_BC3_RGBA = 2,
    QTX_FORMAT_ETC2_RGB = 3,
    QTX_FORMAT_ETC2_RGBA = 4,
    QTX_FORMAT_ASTC_4x4_RGBA = 5,
    QTX_FORMAT_BC7_RGBA = 6,
    QTX_FORMAT_ASTC_6x6_RGBA = 7,
    QTX_FORMAT_ASTC_8x8_RGBA = 8
} qtx_format;

typedef enum qtx_transcode_flags
{
    QTX_TRANSCODE_DEFAULT = 0,
    QTX_TRANSCODE_BUILD_MIPS = 1,
    QTX_TRANSCODE_HIGH_QUALITY = 2
} qtx_transcode_flags;

typedef struct qtx_image_info
{
    uint32_t width;
    uint32_t height;
    uint32_t level_count;
    uint32_t has_alpha;
    uint32_t format_version;
} qtx_image_info;

typedef struct qtx_transcode_info
{
    size_t output_size;
    size_t scratch_size;
    uint32_t level_count;
    uint32_t work_units;
} qtx_transcode_info;

typedef struct qtx_level_info
{
    uint32_t width;
    uint32_t height;
    size_t offset;
    size_t size;
} qtx_level_info;

QTX_API void qtx_init(void);
QTX_API uint32_t qtx_version(void);
QTX_API const char* qtx_result_string(qtx_result result);

QTX_API qtx_result qtx_get_image_info(const void* data, size_t size, qtx_image_info* info);
QTX_API qtx_result qtx_get_transcode_info(const void* data, size_t size, qtx_format format, uint32_t flags, qtx_transcode_info* info);
QTX_API qtx_result qtx_get_level_info(const void* data, size_t size, qtx_format format, uint32_t flags, uint32_t level, qtx_level_info* info);
QTX_API qtx_result qtx_transcode(const void* data, size_t size, qtx_format format, uint32_t flags, void* output, size_t output_size, void* scratch, size_t scratch_size);
QTX_API qtx_result qtx_transcode_units(const void* data, size_t size, qtx_format format, uint32_t flags, uint32_t first_unit, uint32_t unit_count, void* output, size_t output_size, void* scratch, size_t scratch_size);

#ifdef __cplusplus
}
#endif

#endif
