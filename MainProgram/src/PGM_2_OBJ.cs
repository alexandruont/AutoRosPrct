using System.Drawing;
using System.Globalization;
using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Text;
namespace MainProgram.src
{
    static class PGM_2_OBJ
    {
        static float[,] LoadPGM(string path)
        {
            using var fs = new FileStream(path, FileMode.Open, FileAccess.Read);
            using var br = new BinaryReader(fs);

            string ReadToken()
            {
                var sb = new StringBuilder();
                byte b;
                do { b = br.ReadByte(); } while ((char)b == '\n');
                do { sb.Append((char)b); b = br.ReadByte(); } while (!((char)b == '\n'));
                return sb.ToString();
            }

            string magic = ReadToken();
            if (magic != "P5") throw new FormatException("Only binary PGM (P5) format supported.");

            string token;
            do { token = ReadToken(); } while (token.StartsWith("#"));
            string[] dimensions = token.Split(' ');
            int width = int.Parse(dimensions[0]);
            int height = int.Parse(dimensions[1]);
            int maxVal = int.Parse(ReadToken());

            float[,] map = new float[height, width];

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    int value = br.ReadByte();
                    map[y, x] = (float)value / maxVal;
                }
            }

            return map;
        }

        static (List<Vector3> vertices, List<int[]> faces) GenerateMesh(float[,] heightMap)
        {
            int height = heightMap.GetLength(0);
            int width = heightMap.GetLength(1);
            var vertices = new List<Vector3>();
            var faces = new List<int[]>();

            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    vertices.Add(new Vector3(x, heightMap[y, x] * -20f, -y));

            for (int y = 0; y < height - 1; y++)
            {
                for (int x = 0; x < width - 1; x++)
                {
                    int i = y * width + x;
                    int iRight = i + 1;
                    int iBelow = i + width;
                    int iDiagonal = iBelow + 1;

                    faces.Add(new[] { i + 1, iBelow + 1, iRight + 1 });
                    faces.Add(new[] { iRight + 1, iBelow + 1, iDiagonal + 1 });
                }
            }

            return (vertices, faces);
        }

        static void ExportToObj(string path, List<Vector3> vertices, List<int[]> faces)
        {
            using var writer = new StreamWriter(path);

            foreach (var v in vertices)
                writer.WriteLine($"v {v.X:F4} {v.Y:F4} {v.Z:F4}");

            foreach (var f in faces)
                writer.WriteLine($"f {f[0]} {f[1]} {f[2]}");
        }

        static void ExportToPng(string path, float[,] heightMap)
        {
            int height = heightMap.GetLength(0);
            int width = heightMap.GetLength(1);

            using var bmp = new Bitmap(width, height, PixelFormat.Format32bppArgb);

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    int value = (int)(heightMap[y, x] * 255);
                    Color gray;
                    if (value == 205)
                    {
                        gray = Color.Transparent;
                    }
                    else
                    {
                        gray = Color.FromArgb(value, value, value);
                    }
                    bmp.SetPixel(x, y, gray);
                }
            }

            bmp.Save(path, ImageFormat.Png);
        }
    }
}
