using System;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Shouldly;
using Xunit;

namespace UnitTest
{
    public class MathTest
    {
        [Fact(DisplayName = "sweep")]
        public void Sweep()
        {
            // From issue #447
            Sweep sweep = new Sweep();
            sweep.LocalCenter.SetZero();
            sweep.C0.Set(-2.0f, 4.0f);
            sweep.C.Set(3.0f, 8.0f);
            sweep.A0 = 0.5f;
            sweep.A = 5.0f;
            sweep.Alpha0 = 0.0f;

            Transform transform;

            sweep.GetTransform(out transform, 0.0f);
            transform.Position.X.ShouldBe(sweep.C0.X);
            transform.Position.Y.ShouldBe(sweep.C0.Y);
            transform.Rotation.Cos.ShouldBe((float)Math.Cos(sweep.A0));
            transform.Rotation.Sin.ShouldBe((float)Math.Sin(sweep.A0));

            sweep.GetTransform(out transform, 1.0f);
            transform.Position.X.ShouldBe(sweep.C.X);
            transform.Position.Y.ShouldBe(sweep.C.Y);
            transform.Rotation.Cos.ShouldBe((float)Math.Cos(sweep.A));
            transform.Rotation.Sin.ShouldBe((float)Math.Sin(sweep.A));
        }
    }
}