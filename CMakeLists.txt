cmake_minimum_required(VERSION 3.16)
project(qtx VERSION 1.0.0 LANGUAGES C)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(QTX_BUILD_EXAMPLES "Build the C examples" ${PROJECT_IS_TOP_LEVEL})

if(CMAKE_C_COMPILER_ID STREQUAL "MSVC")
    message(FATAL_ERROR "The QTX decoder uses Clang/GCC vector extensions. Use Clang, clang-cl or GCC.")
endif()

add_library(qtx_decoder decoder/src/qtx.c)
add_library(qtx::decoder ALIAS qtx_decoder)
target_include_directories(qtx_decoder PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(qtx_decoder PRIVATE c_std_11)
set_target_properties(qtx_decoder PROPERTIES EXPORT_NAME decoder C_VISIBILITY_PRESET hidden POSITION_INDEPENDENT_CODE ON)
if(BUILD_SHARED_LIBS)
    target_compile_definitions(qtx_decoder PUBLIC QTX_SHARED)
endif()

if(QTX_BUILD_EXAMPLES)
    add_executable(qtx_transcode_example examples/transcode.c)
    target_link_libraries(qtx_transcode_example PRIVATE qtx::decoder)
endif()

install(TARGETS qtx_decoder EXPORT qtxTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES include/qtx.h include/qtx_encoder.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT qtxTargets NAMESPACE qtx:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/qtx)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/qtxConfig.cmake "include(\"\${CMAKE_CURRENT_LIST_DIR}/qtxTargets.cmake\")\n")
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/qtxConfigVersion.cmake COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/qtxConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/qtxConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/qtx)
