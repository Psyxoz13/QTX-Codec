using System;
using System.Buffers.Binary;
using System.IO;
using System.IO.Compression;
using System.Text;

namespace Qtx.Cli;

public static class Png
{
    private static readonly byte[] Signature = { 137, 80, 78, 71, 13, 10, 26, 10 };
    private static readonly uint[] CrcTable = BuildCrcTable();

    public static byte[] ReadRgba(string path, out int width, out int height)
    {
        var data = File.ReadAllBytes(path);
        if (data.Length < Signature.Length || !data.AsSpan(0, Signature.Length).SequenceEqual(Signature))
            throw new InvalidDataException($"{path} is not a PNG file");
        width = height = 0;
        int bitDepth = 0, colorType = 0, interlace = 0;
        byte[] palette = Array.Empty<byte>(), transparency = Array.Empty<byte>();
        using var compressed = new MemoryStream();
        for (var at = Signature.Length; at + 12 <= data.Length;)
        {
            var length = BinaryPrimitives.ReadInt32BigEndian(data.AsSpan(at));
            var type = Encoding.ASCII.GetString(data, at + 4, 4);
            if (length < 0 || at + 12 + length > data.Length)
                throw new InvalidDataException($"{path} has a truncated {type} chunk");
            var body = data.AsSpan(at + 8, length);
            if (type == "IHDR")
            {
                width = BinaryPrimitives.ReadInt32BigEndian(body);
                height = BinaryPrimitives.ReadInt32BigEndian(body[4..]);
                bitDepth = body[8];
                colorType = body[9];
                interlace = body[12];
            }
            else if (type == "PLTE")
                palette = body.ToArray();
            else if (type == "tRNS")
                transparency = body.ToArray();
            else if (type == "IDAT")
                compressed.Write(body);
            else if (type == "IEND")
                break;
            at += 12 + length;
        }
        if (width <= 0 || height <= 0)
            throw new InvalidDataException($"{path} has no image header");
        if (interlace != 0)
            throw new NotSupportedException($"{path} is interlaced, which is not supported");
        var channels = colorType switch { 0 => 1, 2 => 3, 3 => 1, 4 => 2, 6 => 4, _ => throw new InvalidDataException($"{path} has unknown color type {colorType}") };
        var bitsPerPixel = channels * bitDepth;
        var stride = (width * bitsPerPixel + 7) / 8;
        var filterStep = Math.Max(1, bitsPerPixel / 8);
        var rows = new byte[(stride + 1) * height];
        compressed.Position = 0;
        using (var inflater = new ZLibStream(compressed, CompressionMode.Decompress))
            inflater.ReadExactly(rows);
        Unfilter(rows, stride, height, filterStep);
        return ToRgba(rows, width, height, stride, bitDepth, colorType, channels, palette, transparency);
    }

    public static void WriteRgba(string path, byte[] rgba, int width, int height)
    {
        var stride = width * 4;
        var rows = new byte[(stride + 1) * height];
        for (var y = 0; y < height; y++)
            Array.Copy(rgba, y * stride, rows, y * (stride + 1) + 1, stride);
        using var compressed = new MemoryStream();
        using (var deflater = new ZLibStream(compressed, CompressionLevel.SmallestSize, true))
            deflater.Write(rows);
        var header = new byte[13];
        BinaryPrimitives.WriteInt32BigEndian(header, width);
        BinaryPrimitives.WriteInt32BigEndian(header.AsSpan(4), height);
        header[8] = 8;
        header[9] = 6;
        using var output = File.Create(path);
        output.Write(Signature);
        WriteChunk(output, "IHDR", header);
        WriteChunk(output, "IDAT", compressed.ToArray());
        WriteChunk(output, "IEND", Array.Empty<byte>());
    }

    private static void Unfilter(byte[] rows, int stride, int height, int step)
    {
        for (var y = 0; y < height; y++)
        {
            var row = y * (stride + 1) + 1;
            var previous = row - stride - 1;
            var filter = rows[row - 1];
            for (var x = 0; x < stride; x++)
            {
                int left = x >= step ? rows[row + x - step] : 0;
                int up = y > 0 ? rows[previous + x] : 0;
                int upLeft = y > 0 && x >= step ? rows[previous + x - step] : 0;
                rows[row + x] += (byte)(filter switch
                {
                    0 => 0,
                    1 => left,
                    2 => up,
                    3 => (left + up) >> 1,
                    4 => Paeth(left, up, upLeft),
                    _ => throw new InvalidDataException($"PNG row {y} has unknown filter {filter}"),
                });
            }
        }
    }

    private static int Paeth(int left, int up, int upLeft)
    {
        var estimate = left + up - upLeft;
        int toLeft = Math.Abs(estimate - left), toUp = Math.Abs(estimate - up), toUpLeft = Math.Abs(estimate - upLeft);
        return toLeft <= toUp && toLeft <= toUpLeft ? left : toUp <= toUpLeft ? up : upLeft;
    }

    private static byte[] ToRgba(byte[] rows, int width, int height, int stride, int bitDepth, int colorType, int channels, byte[] palette, byte[] transparency)
    {
        var rgba = new byte[width * height * 4];
        var maximum = (1 << bitDepth) - 1;
        Span<int> samples = stackalloc int[4];
        for (var y = 0; y < height; y++)
        {
            var row = y * (stride + 1) + 1;
            for (var x = 0; x < width; x++)
            {
                for (var c = 0; c < channels; c++)
                    samples[c] = Sample(rows, row, x * channels + c, bitDepth);
                var at = (y * width + x) * 4;
                if (colorType == 3)
                {
                    var index = samples[0];
                    if (index * 3 + 2 >= palette.Length)
                        throw new InvalidDataException($"PNG pixel ({x}, {y}) uses palette entry {index} that does not exist");
                    rgba[at] = palette[index * 3];
                    rgba[at + 1] = palette[index * 3 + 1];
                    rgba[at + 2] = palette[index * 3 + 2];
                    rgba[at + 3] = index < transparency.Length ? transparency[index] : (byte)255;
                    continue;
                }
                var gray = colorType == 0 || colorType == 4;
                for (var c = 0; c < 3; c++)
                    rgba[at + c] = Scale(samples[gray ? 0 : c], maximum);
                var hasAlphaChannel = colorType == 4 || colorType == 6;
                rgba[at + 3] = hasAlphaChannel ? Scale(samples[channels - 1], maximum) : IsTransparentKey(samples, gray, bitDepth, transparency) ? (byte)0 : (byte)255;
            }
        }
        return rgba;
    }

    private static int Sample(byte[] rows, int row, int index, int bitDepth)
    {
        if (bitDepth == 16)
            return (rows[row + index * 2] << 8) | rows[row + index * 2 + 1];
        if (bitDepth == 8)
            return rows[row + index];
        var bit = index * bitDepth;
        return (rows[row + bit / 8] >> (8 - bitDepth - bit % 8)) & ((1 << bitDepth) - 1);
    }

    private static byte Scale(int sample, int maximum) => (byte)((sample * 255 + maximum / 2) / maximum);

    private static bool IsTransparentKey(Span<int> samples, bool gray, int bitDepth, byte[] transparency)
    {
        if (transparency.Length < (gray ? 2 : 6))
            return false;
        for (var c = 0; c < (gray ? 1 : 3); c++)
            if (BinaryPrimitives.ReadUInt16BigEndian(transparency.AsSpan(c * 2)) != samples[c])
                return false;
        return true;
    }

    private static void WriteChunk(Stream output, string type, byte[] body)
    {
        Span<byte> number = stackalloc byte[4];
        BinaryPrimitives.WriteInt32BigEndian(number, body.Length);
        output.Write(number);
        var typeBytes = Encoding.ASCII.GetBytes(type);
        output.Write(typeBytes);
        output.Write(body);
        var crc = UpdateCrc(UpdateCrc(0xFFFFFFFFu, typeBytes), body) ^ 0xFFFFFFFFu;
        BinaryPrimitives.WriteUInt32BigEndian(number, crc);
        output.Write(number);
    }

    private static uint UpdateCrc(uint crc, byte[] bytes)
    {
        foreach (var value in bytes)
            crc = CrcTable[(crc ^ value) & 0xFF] ^ (crc >> 8);
        return crc;
    }

    private static uint[] BuildCrcTable()
    {
        var table = new uint[256];
        for (uint n = 0; n < 256; n++)
        {
            var c = n;
            for (var k = 0; k < 8; k++)
                c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[n] = c;
        }
        return table;
    }
}
