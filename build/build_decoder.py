import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / "decoder/src/qtx.c"
INCLUDE = ROOT / "include"
UNITY_EDITOR_DATA = Path(os.environ.get("QTX_UNITY_EDITOR_DATA", "E:/UnityProjects/6000.0.67f1/Editor/Data"))
LLVM = Path(os.environ.get("QTX_LLVM", UNITY_EDITOR_DATA / "PlaybackEngines/WebGLSupport/BuildTools/Emscripten/llvm"))
WASM_LLVM = Path(os.environ.get("QTX_WASM_LLVM", LLVM))
NDK_LLVM = Path(os.environ.get("QTX_NDK_LLVM", UNITY_EDITOR_DATA / "PlaybackEngines/AndroidPlayer/NDK/toolchains/llvm/prebuilt/windows-x86_64/bin"))
COMMON = ["-O3", "-DNDEBUG", "-Wall", "-Wextra", "-DQTX_SHARED", f"-I{INCLUDE}"]
PLATFORMS = ["windows", "linux", "macos", "android", "ios", "wasm"]


def run(*arguments):
    subprocess.run([str(value) for value in arguments], check=True)


def build_windows(output):
    library = output / "windows/x86_64/qtx_decoder.dll"
    library.parent.mkdir(parents=True, exist_ok=True)
    obj = output / "windows.o"
    run(LLVM / "clang.exe", "--target=x86_64-pc-windows-msvc", *COMMON, "-march=x86-64-v2", "-mno-stack-arg-probe", "-ffreestanding", "-DQTX_WITHOUT_LIBC", "-c", SOURCE, "-o", obj)
    run(LLVM / "lld.exe", "-flavor", "link", "-dll", "-noentry", "-nodefaultlib", "-machine:x64", f"-out:{library}", obj)
    obj.unlink()


def build_linux(output):
    library = output / "linux/x86_64/libqtx_decoder.so"
    library.parent.mkdir(parents=True, exist_ok=True)
    obj = output / "linux.o"
    run(LLVM / "clang.exe", "--target=x86_64-unknown-linux-gnu", *COMMON, "-march=x86-64-v2", "-fPIC", "-ffreestanding", "-DQTX_WITHOUT_LIBC", "-c", SOURCE, "-o", obj)
    run(LLVM / "lld.exe", "-flavor", "gnu", "-shared", "-nostdlib", obj, "-o", library)
    obj.unlink()


def build_macos(output):
    folder = output / "macos"
    folder.mkdir(parents=True, exist_ok=True)
    slices = []
    for architecture in ("x86_64", "arm64"):
        obj = folder / f"{architecture}.o"
        library = folder / f"{architecture}.dylib"
        run(NDK_LLVM / "clang.exe", f"--target={architecture}-apple-macos11.0", *COMMON, "-fPIC", "-ffreestanding", "-DQTX_WITHOUT_LIBC", "-c", SOURCE, "-o", obj)
        run(LLVM / "lld.exe", "-flavor", "darwin", "-dylib", "-arch", architecture, "-platform_version", "macos", "11.0", "11.0", "-undefined", "dynamic_lookup", "-install_name", "@rpath/libqtx_decoder.dylib", "-o", library, obj)
        obj.unlink()
        slices.append(library)
    run(NDK_LLVM / "llvm-lipo.exe", "-create", *slices, "-output", folder / "libqtx_decoder.dylib")
    for library in slices:
        library.unlink()


def build_android(output):
    for abi, target in [("arm64-v8a", "aarch64-linux-android21"), ("armeabi-v7a", "armv7a-linux-androideabi21"), ("x86_64", "x86_64-linux-android21")]:
        library = output / "android" / abi / "libqtx_decoder.so"
        library.parent.mkdir(parents=True, exist_ok=True)
        run(NDK_LLVM / "clang.exe", f"--target={target}", *COMMON, "-fPIC", "-shared", "-Wl,-soname,libqtx_decoder.so", "-Wl,-z,max-page-size=16384", "-Wl,-z,common-page-size=16384", SOURCE, "-o", library)


def build_ios(output):
    library = output / "ios/libqtx_decoder.a"
    library.parent.mkdir(parents=True, exist_ok=True)
    obj = output / "ios.o"
    run(NDK_LLVM / "clang.exe", "--target=arm64-apple-ios12.0", *COMMON, "-ffreestanding", "-DQTX_WITHOUT_LIBC", "-c", SOURCE, "-o", obj)
    library.unlink(missing_ok=True)
    run(NDK_LLVM / "llvm-ar.exe", "rcs", library, obj)
    obj.unlink()


def build_wasm(output):
    folder = output / "wasm"
    folder.mkdir(parents=True, exist_ok=True)
    for name, features in (("qtx_decoder.wasm", []), ("qtx_decoder_simd.wasm", ["-msimd128"])):
        run(WASM_LLVM / "clang.exe", "--target=wasm32-unknown-emscripten", "-mcpu=generic", "-mbulk-memory", "-mnontrapping-fptoint", "-msign-ext", *features, *COMMON, "-mllvm", "-unroll-threshold=600", "-nostdlib", "-Wl,--no-entry", "-Wl,-z,stack-size=65536", SOURCE, "-o", folder / name)


def amalgamate(output):
    included = set()

    def expand(path):
        text = path.read_text(encoding="utf-8")

        def replace(match):
            name = match.group(1)
            candidate = path.parent / name if (path.parent / name).exists() else INCLUDE / name
            if candidate in included:
                return ""
            included.add(candidate)
            return expand(candidate)

        return re.sub(r'^#include "([^"]+)"\s*$', replace, text, flags=re.MULTILINE)

    (output / "qtx_decoder.c").write_text(expand(SOURCE), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Build the QTX decoder for every platform")
    parser.add_argument("platforms", nargs="*", metavar="platform", help=f"any of {', '.join(PLATFORMS)}; all when omitted")
    parser.add_argument("--output", type=Path, default=ROOT / "dist")
    arguments = parser.parse_args()
    unknown = sorted(set(arguments.platforms) - set(PLATFORMS))
    if unknown:
        parser.error(f"unknown platforms: {', '.join(unknown)}")
    arguments.output.mkdir(parents=True, exist_ok=True)
    for platform in arguments.platforms or PLATFORMS:
        globals()[f"build_{platform}"](arguments.output)
    amalgamate(arguments.output)
    for path in sorted(arguments.output.rglob("*")):
        if path.is_file():
            print(path.relative_to(arguments.output).as_posix(), path.stat().st_size)


if __name__ == "__main__":
    sys.exit(main())
