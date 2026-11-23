# QTX

Quick TeXture is a compact texture codec that transcodes at load time to BC1, BC3, BC7, ETC2, ASTC, or RGBA. Independent stripes allow multiple threads to work on the same image. The source includes a C decoder, a .NET encoder and CLI, native build scripts, and WebAssembly tooling.

Build the native decoder before building the .NET CLI. On Windows, set QTX_LLVM to the directory containing clang.exe and lld.exe, then run the following commands.

    python build/build_decoder.py windows
    dotnet build tools/Qtx.Cli/Qtx.Cli.csproj

The decoder can also be built with CMake. Use python build/build_decoder.py --help for the platform-specific builds.

[Format and transcoder design](docs/design.md) · [Unity and Addressables integration](https://github.com/Psyxoz13/UniQTX-SDK)


## License

[PolyForm Noncommercial License 1.0.0](LICENSE.md).
