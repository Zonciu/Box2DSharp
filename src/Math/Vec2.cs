using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DSharp
{
    /// <summary>
    /// 2D vector (8 bytes)
    /// This can be used to represent a point or free vector
    /// 2维向量
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 8)]
    public struct Vec2
        : IEquatable<Vec2>
    {
        /// <summary>
        /// X轴值
        /// </summary>
        [FieldOffset(0)]
        public float X;

        /// <summary>
        /// Y轴值
        /// </summary>
        [FieldOffset(4)]
        public float Y;

        /// <summary>
        /// Get the length of this vector (the norm)
        /// 向量长度（法向）
        /// </summary>
        public float Length => MathF.Sqrt(X * X + Y * Y);

        public static readonly Vec2 Zero = new(0, 0);

        public static readonly Vec2 One = new(1, 1);

        public static readonly Vec2 UnitX = new(1, 0);

        public static readonly Vec2 UnitY = new(0, 1);

        /// <summary>
        /// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
        /// 获取单位向量，如果长度为0，则返回零向量
        /// </summary>
        public Vec2 Normalize
        {
            get
            {
                var length = MathF.Sqrt(X * X + Y * Y);
                if (length < float.Epsilon)
                {
                    return Zero;
                }

                var invLength = 1.0f / length;
                Vec2 n = new(invLength * X, invLength * Y);
                return n;
            }
        }

        /// <summary>
        /// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also outputs the length.
        /// 获取单位向量和长度
        /// </summary>
        public (Vec2 Normalize, float Length) GetLengthAndNormalize()
        {
            var length = Length;
            if (length < float.Epsilon)
            {
                return (Zero, length);
            }

            var invLength = 1.0f / length;
            Vec2 n = new(invLength * X, invLength * Y);
            return (n, length);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator Vec2((float X, float Y) tuple)
        {
            return new Vec2(tuple.X, tuple.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator Vector2(Vec2 vec2)
        {
            return new Vector2(vec2.X, vec2.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator Vec2(Vector2 v)
        {
            return new Vec2(v.X, v.Y);
        }

        public Vec2(float x, float y)
        {
            X = x;
            Y = y;
        }

        public void Set(float x, float y)
        {
            X = x;
            Y = y;
        }

        public override string ToString()
        {
            return $"({X},{Y})";
        }

        public bool IsValid()
        {
            if (float.IsNaN(X) || float.IsNaN(Y))
            {
                return false;
            }

            if (float.IsInfinity(X) || float.IsInfinity(Y))
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// Unary negate a vector
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 operator -(in Vec2 a) => new(-a.X, -a.Y);

        /// <summary>
        /// Binary vector addition
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 operator +(in Vec2 a, in Vec2 b) => new(a.X + b.X, a.Y + b.Y);

        /// <summary>
        /// Binary vector subtraction
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 operator -(in Vec2 a, in Vec2 b) => new(a.X - b.X, a.Y - b.Y);

        /// <summary>
        /// Binary scalar and vector multiplication
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 operator *(float a, in Vec2 b) => new(a * b.X, a * b.Y);

        /// <summary>
        /// Binary scalar and vector multiplication
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2 operator *(in Vec2 a, float b) => new(a.X * b, a.Y * b);

        /// <summary>
        /// Binary vector equality
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(in Vec2 a, in Vec2 b) => a.X.Equals(b.X) && a.Y.Equals(b.Y);

        /// <summary>
        /// Binary vector inequality
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(in Vec2 a, in Vec2 b) => !(a == b);

        public bool Equals(Vec2 other)
        {
            return X.Equals(other.X)
                && Y.Equals(other.Y);
        }

        public override bool Equals(object? obj)
        {
            return obj is Vec2 other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(X, Y);
        }
    }
}