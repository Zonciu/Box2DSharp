using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Determinism;

[Sample("Determinism", "Falling Hinges")]
public class FallingHinges : SampleBase
{
    // This sample provides a visual representation of the cross platform determinism unit test.
    // The scenario is designed to produce a chaotic result engaging:
    // - continuous collision
    // - joint limits (approximate atan2)
    // - MakeRot (approximate sin/cos)
    // Once all the bodies go to sleep the step counter and transform hash is emitted which
    // can then be transferred to the unit test and tested in GitHub build actions.
    // See CrossPlatformTest in the unit tests.

    private const int e_columns = 4;

    private const int e_rows = 30;

    BodyId[] m_bodies = new BodyId[e_rows * e_columns];

    uint m_hash;

    int m_sleepStep;

    public FallingHinges(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 7.5f);
            Global.Camera.Zoom = 10.0f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (0.0f, -1.0f);
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            Polygon b = Geometry.MakeBox(20.0f, 1.0f);
            ShapeDef s = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(groundId, s, b);
        }

        for (int i = 0; i < e_rows * e_columns; ++i)
        {
            m_bodies[i] = BodyId.NullId;
        }

        float h = 0.25f;
        float r = 0.1f * h;
        Polygon box = Geometry.MakeRoundedBox(h - r, h - r, r);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Friction = 0.3f;

        float offset = 0.4f * h;
        float dx = 10.0f * h;
        float xroot = -0.5f * dx * (e_columns - 1.0f);

        RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
        jointDef.EnableLimit = true;
        jointDef.LowerAngle = -0.1f * B2Math.Pi;
        jointDef.UpperAngle = 0.2f * B2Math.Pi;
        jointDef.EnableSpring = true;
        jointDef.Hertz = 0.5f;
        jointDef.DampingRatio = 0.5f;
        jointDef.LocalAnchorA = (h, h);
        jointDef.LocalAnchorB = (offset, -h);
        jointDef.DrawSize = 0.1f;

        int bodyIndex = 0;
        int bodyCount = e_rows * e_columns;

        for (int j = 0; j < e_columns; ++j)
        {
            float x = xroot + j * dx;

            BodyId prevBodyId = BodyId.NullId;

            for (int i = 0; i < e_rows; ++i)
            {
                BodyDef bodyDef = BodyDef.DefaultBodyDef();
                bodyDef.Type = BodyType.DynamicBody;

                bodyDef.Position.X = x + offset * i;
                bodyDef.Position.Y = h + 2.0f * h * i;

                // this tests the deterministic cosine and sine functions
                bodyDef.Rotation = B2Math.MakeRot(0.1f * i - 1.0f);

                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

                if ((i & 1) == 0)
                {
                    prevBodyId = bodyId;
                }
                else
                {
                    jointDef.BodyIdA = prevBodyId;
                    jointDef.BodyIdB = bodyId;
                    Joint.CreateRevoluteJoint(WorldId, jointDef);
                    prevBodyId = BodyId.NullId;
                }

                Shape.CreatePolygonShape(bodyId, shapeDef, box);

                Debug.Assert(bodyIndex < bodyCount);
                m_bodies[bodyIndex] = bodyId;

                bodyIndex += 1;
            }
        }

        m_hash = 0;
        m_sleepStep = -1;

        //PrintTransforms();
    }

    void PrintTransforms()
    {
        uint hash = HashTool.HashInit;
        int bodyCount = e_rows * e_columns;
        for (int i = 0; i < bodyCount; ++i)
        {
            Transform xf = Body.GetTransform(m_bodies[i]);
            Console.WriteLine("{0} {1} {2} {3} {4}", i, xf.P.X, xf.P.Y, xf.Q.C, xf.Q.S);
            unsafe
            {
                var r = MemoryMarshal.CreateSpan(ref Unsafe.AsRef<byte>(Unsafe.AsPointer(ref xf)), 16);
                hash = HashTool.Hash(hash, r);
            }
        }

        Console.WriteLine("hash = 0x{0:X8}", hash);
    }

    protected override void OnRender()
    {
        base.OnRender();

        if (m_hash == 0)
        {
            bool sleeping = true;
            int bodyCount = e_rows * e_columns;
            for (int i = 0; i < bodyCount; ++i)
            {
                if (Body.IsAwake(m_bodies[i]))
                {
                    sleeping = false;
                    break;
                }
            }

            if (sleeping)
            {
                var hash = HashTool.HashInit;
                for (int i = 0; i < bodyCount; ++i)
                {
                    Transform xf = Body.GetTransform(m_bodies[i]);

                    //printf( "%d %.9f %.9f %.9f %.9f\n", i, xf.P.x, xf.P.y, xf.q.c, xf.q.s );
                    unsafe
                    {
                        var r = MemoryMarshal.CreateSpan(ref Unsafe.AsRef<byte>(Unsafe.AsPointer(ref xf)), 16);
                        hash = HashTool.Hash(hash, r);
                    }
                }

                m_sleepStep = StepCount - 1;
                m_hash = hash;
                Console.WriteLine("sleep step = {0}, hash = 0x{1:X8}", m_sleepStep, m_hash);
            }
        }

        DrawString($"sleep step = {m_sleepStep}, hash = 0x{m_hash:X8}");
    }
}