using System;
using System.IO;
using System.Runtime.InteropServices;

namespace Qtx.Cli;

public enum QtxFormat
{
    Rgba8 = 0,
    Bc1Rgb = 1,
    Bc3Rgba = 2,
    Etc2Rgb = 3,
    Etc2Rgba = 4,
    Astc4x4Rgba = 5,
    Bc7Rgba = 6,
}

public static class QtxFile
{
    public static QtxNativeDecoder.ImageInfo ReadInfo(byte[] file)
    {
        Check(QtxNativeDecoder.qtx_get_image_info(file, (nuint)file.Length, out var info));
        return info;
    }

    public static byte[] DecodeFirstLevelRgba(byte[] file)
    {
        var output = Transcode(file, QtxFormat.Rgba8, 0);
        Check(QtxNativeDecoder.qtx_get_level_info(file, (nuint)file.Length, (int)QtxFormat.Rgba8, 0, 0, out var level));
        return output.AsSpan((int)level.Offset, (int)level.Size).ToArray();
    }

    public static byte[] Transcode(byte[] file, QtxFormat format, uint flags)
    {
        Check(QtxNativeDecoder.qtx_get_transcode_info(file, (nuint)file.Length, (int)format, flags, out var info));
        var output = new byte[info.OutputSize];
        var scratch = new byte[info.ScratchSize];
        Check(QtxNativeDecoder.qtx_transcode(file, (nuint)file.Length, (int)format, flags, output, (nuint)output.Length, scratch, (nuint)scratch.Length));
        return output;
    }

    private static void Check(int result)
    {
        if (result != 0)
            throw new InvalidDataException(Marshal.PtrToStringUTF8(QtxNativeDecoder.qtx_result_string(result)));
    }
}

public static class QtxNativeDecoder
{
    private const string Library = "qtx_decoder";

    [StructLayout(LayoutKind.Sequential)]
    public struct ImageInfo
    {
        public uint Width;
        public uint Height;
        public uint LevelCount;
        public uint HasAlpha;
        public uint FormatVersion;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct TranscodeInfo
    {
        public nuint OutputSize;
        public nuint ScratchSize;
        public uint LevelCount;
        public uint WorkUnits;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct LevelInfo
    {
        public uint Width;
        public uint Height;
        public nuint Offset;
        public nuint Size;
    }

    [DllImport(Library)] public static extern IntPtr qtx_result_string(int result);
    [DllImport(Library)] public static extern int qtx_get_image_info(byte[] data, nuint size, out ImageInfo info);
    [DllImport(Library)] public static extern int qtx_get_transcode_info(byte[] data, nuint size, int format, uint flags, out TranscodeInfo info);
    [DllImport(Library)] public static extern int qtx_get_level_info(byte[] data, nuint size, int format, uint flags, uint level, out LevelInfo info);
    [DllImport(Library)] public static extern int qtx_transcode(byte[] data, nuint size, int format, uint flags, byte[] output, nuint outputSize, byte[] scratch, nuint scratchSize);
}
