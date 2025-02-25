using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Benchmark;

[Sample("Benchmark", "JointGrid")]
public class JointGrid : SampleBase
{
    protected float Gravity;

    public JointGrid(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (60.0f, -57.0f);
            Global.Camera.Zoom = 25.0f * 2.5f;
        }

        const int N = Core.B2Debug ? 10 : 100;

        // Allocate to avoid huge stack usage
        BodyId[] bodies = new BodyId[N * N];
        int index = 0;

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;
        shapeDef.Filter.CategoryBits = 2;
        shapeDef.Filter.MaskBits = ~2u;

        Circle circle = ((0.0f, 0.0f), 0.4f);

        RevoluteJointDef jd = RevoluteJointDef.DefaultRevoluteJointDef();
        BodyDef bodyDef = BodyDef.DefaultBodyDef();

        for (int k = 0; k < N; ++k)
        {
            for (int i = 0; i < N; ++i)
            {
                float fk = k;
                float fi = i;

                if (k >= N / 2 - 3 && k <= N / 2 + 3 && i == 0)
                {
                    bodyDef.Type = BodyType.StaticBody;
                }
                else
                {
                    bodyDef.Type = BodyType.DynamicBody;
                }

                bodyDef.Position = (fk, -fi);

                BodyId body = Body.CreateBody(WorldId, bodyDef);

                Shape.CreateCircleShape(body, shapeDef, circle);

                if (i > 0)
                {
                    jd.BodyIdA = bodies[index - 1];
                    jd.BodyIdB = body;
                    jd.LocalAnchorA = (0.0f, -0.5f);
                    jd.LocalAnchorB = (0.0f, 0.5f);
                    Joint.CreateRevoluteJoint(WorldId, jd);
                }

                if (k > 0)
                {
                    jd.BodyIdA = bodies[index - N];
                    jd.BodyIdB = body;
                    jd.LocalAnchorA = (0.5f, 0.0f);
                    jd.LocalAnchorB = (-0.5f, 0.0f);
                    Joint.CreateRevoluteJoint(WorldId, jd);
                }

                bodies[index++] = body;
            }
        }

        Gravity = 10.0f;
    }
}