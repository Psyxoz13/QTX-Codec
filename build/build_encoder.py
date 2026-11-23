import argparse
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PROJECT = ROOT / "encoder/Qtx.Encoder.Native/Qtx.Encoder.Native.csproj"
VISUAL_STUDIO_INSTALLER = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer"
LIBRARY_EXTENSIONS = {"win": ".dll", "linux": ".so", "osx": ".dylib"}
AVX2_VARIANT = ("qtx_encoder_avx2", "x86-64-v3")


def host_runtime():
    system = {"Windows": "win", "Linux": "linux", "Darwin": "osx"}[platform.system()]
    machine = "arm64" if platform.machine().lower() in ("arm64", "aarch64") else "x64"
    return f"{system}-{machine}"


def main():
    parser = argparse.ArgumentParser(description="Publish the QTX encoder as a native library with NativeAOT (only for the host operating system)")
    parser.add_argument("--runtime", default=host_runtime(), help="runtime identifier, for example win-x64, linux-x64, osx-arm64")
    parser.add_argument("--output", type=Path, default=ROOT / "dist/encoder")
    arguments = parser.parse_args()
    environment = dict(os.environ)
    if VISUAL_STUDIO_INSTALLER.exists():
        environment["PATH"] = str(VISUAL_STUDIO_INSTALLER) + os.pathsep + environment["PATH"]
    variants = [("qtx_encoder", None)]
    if arguments.runtime.endswith("-x64"):
        variants.append(AVX2_VARIANT)
    for assembly_name, instruction_set in variants:
        publish = ROOT / "encoder/Qtx.Encoder.Native/bin/publish" / f"{arguments.runtime}-{assembly_name}"
        command = ["dotnet", "publish", str(PROJECT), "-c", "Release", "-r", arguments.runtime, "-o", str(publish), f"-p:QtxEncoderName={assembly_name}"]
        if instruction_set is not None:
            command.append(f"-p:IlcInstructionSet={instruction_set}")
        subprocess.run(command, check=True, env=environment)
        name = assembly_name + LIBRARY_EXTENSIONS[arguments.runtime.split("-")[0]]
        destination = arguments.output / arguments.runtime / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(publish / name, destination)
        print(destination.relative_to(ROOT).as_posix(), destination.stat().st_size)


if __name__ == "__main__":
    sys.exit(main())
