using System.Numerics;
using Box2DSharp.Common;
using Box2DSharp.Ropes;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Rope", "Bending")]
    public class RopeTest : TestBase
    {
        protected readonly Rope Rope1;

        protected readonly Rope Rope2;

        protected readonly RopeTuning Tuning1;

        protected readonly RopeTuning Tuning2;

        protected int Iterations1;

        protected int Iterations2;

        protected Vector2 Position1;

        protected Vector2 Position2;

        protected float Speed;

        public RopeTest()
        {
            const int N = 20;
            const float L = 0.5f;
            var vertices = new Vector2[N];
            var masses = new float[N];

            for (var i = 0; i < N; ++i)
            {
                vertices[i].Set(0.0f, L * (N - i));
                masses[i] = 1.0f;
            }

            masses[0] = 0.0f;
            masses[1] = 0.0f;
            Tuning1 = new RopeTuning
            {
                BendHertz = 30.0f,
                BendDamping = 4.0f,
                BendStiffness = 1.0f,
                BendingModel = BendingModel.PbdTriangleBendingModel,
                Isometric = true,
                StretchHertz = 30.0f,
                StretchDamping = 4.0f,
                StretchStiffness = 1.0f,
                StretchingModel = StretchingModel.PbdStretchingModel
            };

            Tuning2 = new RopeTuning
            {
                BendHertz = 30.0f,
                BendDamping = 0.7f,
                BendStiffness = 1.0f,
                BendingModel = BendingModel.PbdHeightBendingModel,
                Isometric = true,
                StretchHertz = 30.0f,
                StretchDamping = 1.0f,
                StretchStiffness = 1.0f,
                StretchingModel = StretchingModel.PbdStretchingModel
            };

            Position1.Set(-5.0f, 15.0f);
            Position2.Set(5.0f, 15.0f);

            var def = new RopeDef
            {
                Vertices = vertices,
                Count = N,
                Gravity = new Vector2(0.0f, -10.0f),
                Masses = masses,
                Position = Position1,
                Tuning = Tuning1
            };
            Rope1 = new Rope();
            Rope1.Create(def);

            def.Position = Position2;
            def.Tuning = Tuning2;
            Rope2 = new Rope();
            Rope2.Create(def);

            Iterations1 = 8;
            Iterations2 = 8;

            Speed = 10.0f;
        }

        protected override void PreStep()
        {
            var dt = TestSettings.Hertz > 0.0f ? 1.0f / TestSettings.Hertz : 0.0f;
            if (Input.IsKeyDown(KeyCodes.Comma))
            {
                Position1.X -= Speed * dt;
                Position2.X -= Speed * dt;
            }

            if (Input.IsKeyDown(KeyCodes.Period))
            {
                Position1.X += Speed * dt;
                Position2.X += Speed * dt;
            }

            if (TestSettings.Pause && !TestSettings.SingleStep)
            {
                dt = 0.0f;
            }

            Rope1.SetTuning(Tuning1);
            Rope2.SetTuning(Tuning2);
            Rope1.Step(dt, Iterations1, Position1);
            Rope2.Step(dt, Iterations2, Position2);
        }
    }
}