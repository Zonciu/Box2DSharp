using System.Runtime.InteropServices;

namespace Box2DSharp
{
    /// <summary>
    /// A 2D rigid transform (16 bytes)
    /// 位移描述，含坐标和角度(弧度制)
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 16)]
    public struct Transform
    {
        [FieldOffset(0)]
        public Vec2 P;

        [FieldOffset(8)]
        public Rot Q;

        public Transform(Vec2 p, Rot q)
        {
            P = p;
            Q = q;
        }

        public static implicit operator Transform((Vec2 p, Rot q) tuple)
        {
            return new Transform(tuple.p, tuple.q);
        }

        public static readonly Transform Identity = new(new(0, 0), new(1, 0));

        public override string ToString()
        {
            return $"{P}({Q})";
        }
    }
}