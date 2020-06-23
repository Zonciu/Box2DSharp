using System;
using System.Drawing;
using System.Drawing.Imaging;
using OpenToolkit.Graphics.OpenGL4;
using PixelFormat = OpenToolkit.Graphics.OpenGL4.PixelFormat;

namespace Testbed.Basics.ImGui
{
    public enum TextureCoordinate
    {
        S = TextureParameterName.TextureWrapS,

        T = TextureParameterName.TextureWrapT,

        R = TextureParameterName.TextureWrapR
    }

    class Texture : IDisposable
    {
        public const SizedInternalFormat Srgb8Alpha8 = (SizedInternalFormat)All.Srgb8Alpha8;

        public const SizedInternalFormat Rgb32F = (SizedInternalFormat)All.Rgb32f;

        public const GetPName MaxTextureMaxAnisotropy = (GetPName)0x84FF;

        public static readonly float MaxAniso;

        static Texture()
        {
            MaxAniso = GL.GetFloat(MaxTextureMaxAnisotropy);
        }

        public readonly string Name;

        public readonly int GLTexture;

        public readonly int Width;

        public readonly int Height;

        public readonly int MipmapLevels;

        public readonly SizedInternalFormat InternalFormat;

        public Texture(string name, Bitmap image, bool generateMipmaps, bool srgb)
        {
            Name = name;
            Width = image.Width;
            Height = image.Height;
            InternalFormat = srgb ? Srgb8Alpha8 : SizedInternalFormat.Rgba8;

            if (generateMipmaps)
            {
                // Calculate how many levels to generate for this texture
                MipmapLevels = (int)Math.Floor(Math.Log(Math.Max(Width, Height), 2));
            }
            else
            {
                // There is only one level
                MipmapLevels = 1;
            }

            Util.CheckGLError("Clear");

            Util.CreateTexture(TextureTarget.Texture2D, Name, out GLTexture);
            GL.TextureStorage2D(GLTexture, MipmapLevels, InternalFormat, Width, Height);
            Util.CheckGLError("Storage2d");

            var data = image.LockBits(
                new Rectangle(0, 0, Width, Height),
                ImageLockMode.ReadOnly,
                global::System.Drawing.Imaging.PixelFormat.Format32bppArgb);

            GL.TextureSubImage2D(
                GLTexture,
                0,
                0,
                0,
                Width,
                Height,
                PixelFormat.Bgra,
                PixelType.UnsignedByte,
                data.Scan0);
            Util.CheckGLError("SubImage");

            image.UnlockBits(data);

            if (generateMipmaps)
            {
                GL.GenerateTextureMipmap(GLTexture);
            }

            GL.TextureParameter(GLTexture, TextureParameterName.TextureWrapS, (int)TextureWrapMode.Repeat);
            Util.CheckGLError("WrapS");
            GL.TextureParameter(GLTexture, TextureParameterName.TextureWrapT, (int)TextureWrapMode.Repeat);
            Util.CheckGLError("WrapT");

            GL.TextureParameter(GLTexture, TextureParameterName.TextureMinFilter, (int)(generateMipmaps ? TextureMinFilter.Linear : TextureMinFilter.LinearMipmapLinear));
            GL.TextureParameter(GLTexture, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
            Util.CheckGLError("Filtering");

            GL.TextureParameter(GLTexture, TextureParameterName.TextureMaxLevel, MipmapLevels - 1);

            // This is a bit weird to do here
            image.Dispose();
        }

        public Texture(string name, int GLTex, int width, int height, int mipmaplevels, SizedInternalFormat internalFormat)
        {
            Name = name;
            GLTexture = GLTex;
            Width = width;
            Height = height;
            MipmapLevels = mipmaplevels;
            InternalFormat = internalFormat;
        }

        public Texture(string name, int width, int height, IntPtr data, bool generateMipmaps = false, bool srgb = false)
        {
            Name = name;
            Width = width;
            Height = height;
            InternalFormat = srgb ? Srgb8Alpha8 : SizedInternalFormat.Rgba8;
            MipmapLevels = generateMipmaps == false ? 1 : (int)Math.Floor(Math.Log(Math.Max(Width, Height), 2));

            Util.CreateTexture(TextureTarget.Texture2D, Name, out GLTexture);
            GL.TextureStorage2D(GLTexture, MipmapLevels, InternalFormat, Width, Height);

            GL.TextureSubImage2D(
                GLTexture,
                0,
                0,
                0,
                Width,
                Height,
                PixelFormat.Bgra,
                PixelType.UnsignedByte,
                data);

            if (generateMipmaps)
            {
                GL.GenerateTextureMipmap(GLTexture);
            }

            SetWrap(TextureCoordinate.S, TextureWrapMode.Repeat);
            SetWrap(TextureCoordinate.T, TextureWrapMode.Repeat);

            GL.TextureParameter(GLTexture, TextureParameterName.TextureMaxLevel, MipmapLevels - 1);
        }

        public void SetMinFilter(TextureMinFilter filter)
        {
            GL.TextureParameter(GLTexture, TextureParameterName.TextureMinFilter, (int)filter);
        }

        public void SetMagFilter(TextureMagFilter filter)
        {
            GL.TextureParameter(GLTexture, TextureParameterName.TextureMagFilter, (int)filter);
        }

        public void SetAnisotropy(float level)
        {
            const TextureParameterName TEXTURE_MAX_ANISOTROPY = (TextureParameterName)0x84FE;
            GL.TextureParameter(GLTexture, TEXTURE_MAX_ANISOTROPY, Math.Clamp(level, 1, MaxAniso));
        }

        public void SetLod(int @base, int min, int max)
        {
            GL.TextureParameter(GLTexture, TextureParameterName.TextureLodBias, @base);
            GL.TextureParameter(GLTexture, TextureParameterName.TextureMinLod, min);
            GL.TextureParameter(GLTexture, TextureParameterName.TextureMaxLod, max);
        }

        public void SetWrap(TextureCoordinate coord, TextureWrapMode mode)
        {
            GL.TextureParameter(GLTexture, (TextureParameterName)coord, (int)mode);
        }

        public void Dispose()
        {
            GL.DeleteTexture(GLTexture);
        }
    }
}