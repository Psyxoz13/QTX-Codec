using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;

namespace Qtx.Cli;

public static class Program
{
    private const string Usage = @"qtx - QTX texture codec

  qtx encode <input.png> <output.qtx> [--quality 0-100] [--mips]
  qtx decode <input.qtx> <output.png>
  qtx info <input.qtx>

Quality defaults to 50. --mips stores the full mip chain in the file.";

    public static int Main(string[] args)
    {
        try
        {
            return args.Length switch
            {
                >= 3 when args[0] == "encode" => Encode(args[1], args[2], args[3..]),
                3 when args[0] == "decode" => Decode(args[1], args[2]),
                2 when args[0] == "info" => Info(args[1]),
                _ => Fail(Usage),
            };
        }
        catch (Exception exception) when (exception is IOException or InvalidDataException or NotSupportedException or ArgumentException or UnauthorizedAccessException)
        {
            return Fail(exception.Message);
        }
    }

    private static int Encode(string input, string output, string[] options)
    {
        var encodeOptions = new QtxEncodeOptions();
        for (var i = 0; i < options.Length; i++)
        {
            if (options[i] == "--mips")
                encodeOptions.Mipmaps = true;
            else if (options[i] == "--quality" && i + 1 < options.Length && int.TryParse(options[i + 1], NumberStyles.Integer, CultureInfo.InvariantCulture, out var quality) && quality is >= QtxEncodeOptions.MinQuality and <= QtxEncodeOptions.MaxQuality)
            {
                encodeOptions.Quality = quality;
                i++;
            }
            else
                return Fail($"unknown or invalid option '{options[i]}'\n\n{Usage}");
        }
        var rgba = Png.ReadRgba(input, out var width, out var height);
        if (width > QtxEncoder.MaxDimension || height > QtxEncoder.MaxDimension)
            return Fail($"{input} is {width}x{height}; QTX stores up to {QtxEncoder.MaxDimension} pixels per side");
        var stopwatch = Stopwatch.StartNew();
        var encoded = QtxEncoder.Encode(rgba, width, height, encodeOptions);
        File.WriteAllBytes(output, encoded);
        Console.WriteLine($"{input}: {width}x{height} -> {output}: {encoded.Length} bytes ({encoded.Length * 8.0 / (width * height):F2} bits per pixel) in {stopwatch.Elapsed.TotalMilliseconds:F0} ms");
        return 0;
    }

    private static int Decode(string input, string output)
    {
        var file = File.ReadAllBytes(input);
        var info = QtxFile.ReadInfo(file);
        Png.WriteRgba(output, QtxFile.DecodeFirstLevelRgba(file), (int)info.Width, (int)info.Height);
        Console.WriteLine($"{input} -> {output}: {info.Width}x{info.Height}");
        return 0;
    }

    private static int Info(string input)
    {
        var file = File.ReadAllBytes(input);
        var info = QtxFile.ReadInfo(file);
        Console.WriteLine($"{input}");
        Console.WriteLine($"  size:    {info.Width}x{info.Height}, {info.LevelCount} stored mip level(s)");
        Console.WriteLine($"  format:  QTX version {info.FormatVersion}, {(info.HasAlpha != 0 ? "with" : "without")} alpha");
        Console.WriteLine($"  bytes:   {file.Length} ({file.Length * 8.0 / ((double)info.Width * info.Height):F2} bits per pixel of level 0)");
        return 0;
    }

    private static int Fail(string message)
    {
        Console.Error.WriteLine(message);
        return 1;
    }
}
