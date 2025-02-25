using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Collision;

[Sample("Collision", "Time Of Impact")]
public class TimeOfImpact : SampleBase
{
    Vec2[] m_verticesA = [(-1.0f, -1.0f), (1.0f, -1.0f), (1.0f, 5.0f), (-1.0f, 5.0f)];

    Vec2[] m_verticesB = [(-0.5f, -4.0f), (0.0f, -4.0f), (0.0f, 0.0f), (-0.5f, 0.0f)];

    int m_countA = 4;

    int m_countB = 4;

    public TimeOfImpact(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.6f, 2.0f);
            Global.Camera.Zoom = 25.0f * 0.18f;
        }
    }

    protected override void OnRender()
    {
        base.OnRender();

        Sweep sweepA = (Vec2.Zero, (0.0f, 0.0f), (0.0f, 0.0f), Rot.Identity, Rot.Identity);
        Sweep sweepB = (Vec2.Zero, (2.0f, 4.0f), (2.0f, 4.0f), Rot.Identity, B2Math.MakeRot(-0.25f * B2Math.Pi));

        TOIInput input;
        input.ProxyA = DistanceFunc.MakeProxy(m_verticesA, m_countA, 0.0f);
        input.ProxyB = DistanceFunc.MakeProxy(m_verticesB, m_countB, 0.0f);
        input.SweepA = sweepA;
        input.SweepB = sweepB;
        input.TMax = 1.0f;

        TOIOutput output = DistanceFunc.TimeOfImpact(input);

        DrawString($"toi = {output.T:G}");

        // Draw.DrawString(5, m_textLine, "max toi iters = %d, max root iters = %d", _toiMaxIters,
        //                        _toiMaxRootIters);

        Span<Vec2> vertices = stackalloc Vec2[Core.MaxPolygonVertices];

        // Draw A
        Transform transformA = DistanceFunc.GetSweepTransform(sweepA, 0.0f);
        for (int i = 0; i < m_countA; ++i)
        {
            vertices[i] = B2Math.TransformPoint(transformA, m_verticesA[i]);
        }

        Draw.DrawPolygon(vertices, m_countA, B2HexColor.Gray);

        // Draw B at t = 0
        Transform transformB = DistanceFunc.GetSweepTransform(sweepB, 0.0f);
        for (int i = 0; i < m_countB; ++i)
        {
            vertices[i] = B2Math.TransformPoint(transformB, m_verticesB[i]);
        }

        Draw.DrawPolygon(vertices, m_countB, B2HexColor.Green);

        // Draw B at t = hit_time
        transformB = DistanceFunc.GetSweepTransform(sweepB, output.T);
        for (int i = 0; i < m_countB; ++i)
        {
            vertices[i] = B2Math.TransformPoint(transformB, m_verticesB[i]);
        }

        Draw.DrawPolygon(vertices, m_countB, B2HexColor.Orange);

        // Draw B at t = 1
        transformB = DistanceFunc.GetSweepTransform(sweepB, 1.0f);
        for (int i = 0; i < m_countB; ++i)
        {
            vertices[i] = B2Math.TransformPoint(transformB, m_verticesB[i]);
        }

        Draw.DrawPolygon(vertices, m_countB, B2HexColor.Red);

        if (output.State == TOIState.Hit)
        {
            DistanceInput dinput;
            dinput.ProxyA = input.ProxyA;
            dinput.ProxyB = input.ProxyB;
            dinput.TransformA = DistanceFunc.GetSweepTransform(sweepA, output.T);
            dinput.TransformB = DistanceFunc.GetSweepTransform(sweepB, output.T);
            dinput.UseRadii = false;
            DistanceCache cache = new();
            DistanceOutput doutput = DistanceFunc.ShapeDistance(ref cache, dinput, null, 0);
            DrawString($"distance = {doutput.Distance:G}");
        }
    }
}