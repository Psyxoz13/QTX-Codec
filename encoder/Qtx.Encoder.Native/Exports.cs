using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Qtx;

[StructLayout(LayoutKind.Sequential)]
public struct QtxEncodeParams
{
    public const uint MipsFlag = 1;

    public uint StructSize;
    public int Quality;
    public uint Flags;
}

public static unsafe class Exports
{
    private const int Success = 0;
    private const int InvalidArgument = -1;
    private const int Internal = -7;

    [UnmanagedCallersOnly(EntryPoint = "qtx_encode", CallConvs = new[] { typeof(CallConvCdecl) })]
    public static int Encode(byte* rgba, uint width, uint height, QtxEncodeParams* parameters, byte** output, nuint* outputSize)
    {
        if (output == null || outputSize == null)
            return InvalidArgument;
        *output = null;
        *outputSize = 0;
        if (rgba == null || !IsDimension(width) || !IsDimension(height) || !TryReadOptions(parameters, out var options))
            return InvalidArgument;
        var length = (long)width * height * 4;
        if (length > int.MaxValue)
            return InvalidArgument;
        return Run(() => QtxEncoder.Encode(new ReadOnlySpan<byte>(rgba, (int)length), (int)width, (int)height, options), output, outputSize);
    }

    [UnmanagedCallersOnly(EntryPoint = "qtx_encode_levels", CallConvs = new[] { typeof(CallConvCdecl) })]
    public static int EncodeLevels(byte* levels, uint levelCount, uint width, uint height, QtxEncodeParams* parameters, byte** output, nuint* outputSize)
    {
        if (output == null || outputSize == null)
            return InvalidArgument;
        *output = null;
        *outputSize = 0;
        if (levels == null || levelCount < 1 || levelCount > QtxEncoder.MaxLevels || !IsDimension(width) || !IsDimension(height) || !TryReadOptions(parameters, out var options))
            return InvalidArgument;
        var images = new byte[levelCount][];
        long offset = 0;
        for (var level = 0; level < levelCount; level++)
        {
            var length = (long)Math.Max(1, (int)width >> level) * Math.Max(1, (int)height >> level) * 4;
            if (length > int.MaxValue)
                return InvalidArgument;
            images[level] = new ReadOnlySpan<byte>(levels + offset, (int)length).ToArray();
            offset += length;
        }
        return Run(() => QtxEncoder.EncodeLevels(images, (int)width, (int)height, options), output, outputSize);
    }

    [UnmanagedCallersOnly(EntryPoint = "qtx_free", CallConvs = new[] { typeof(CallConvCdecl) })]
    public static void Free(void* memory)
    {
        NativeMemory.Free(memory);
    }

    private static bool IsDimension(uint value) => value >= 1 && value <= QtxEncoder.MaxDimension;

    private static bool TryReadOptions(QtxEncodeParams* parameters, out QtxEncodeOptions options)
    {
        options = new QtxEncodeOptions();
        if (parameters == null)
            return true;
        if (parameters->StructSize < sizeof(QtxEncodeParams) || (parameters->Flags & ~QtxEncodeParams.MipsFlag) != 0)
            return false;
        if (parameters->Quality < QtxEncodeOptions.MinQuality || parameters->Quality > QtxEncodeOptions.MaxQuality)
            return false;
        options.Quality = parameters->Quality;
        options.Mipmaps = (parameters->Flags & QtxEncodeParams.MipsFlag) != 0;
        return true;
    }

    private static int Run(Func<byte[]> encode, byte** output, nuint* outputSize)
    {
        byte[] bytes;
        try
        {
            bytes = encode();
        }
        catch (ArgumentException)
        {
            return InvalidArgument;
        }
        catch (Exception)
        {
            return Internal;
        }
        var result = (byte*)NativeMemory.Alloc((nuint)bytes.Length);
        bytes.AsSpan().CopyTo(new Span<byte>(result, bytes.Length));
        *output = result;
        *outputSize = (nuint)bytes.Length;
        return Success;
    }
}
