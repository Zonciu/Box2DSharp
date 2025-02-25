using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Benchmark;

[Sample("Benchmark", "Sleep")]
public class Sleep : SampleBase
{
    private const int e_maxBaseCount = 100;

    private const int e_maxBodyCount = e_maxBaseCount * (e_maxBaseCount + 1) / 2;

    BodyId[] m_bodies = new BodyId[e_maxBodyCount];

    int m_bodyCount;

    int m_baseCount;

    int m_iterations;

    float m_wakeTotal;

    float m_sleepTotal;

    int m_wakeCount;

    int m_sleepCount;

    bool m_awake;

    public Sleep(Settings settings)
        : base(settings)

    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 50.0f);
            Global.Camera.Zoom = 25.0f * 2.2f;
        }

        float groundSize = 100.0f;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        BodyId groundId = Body.CreateBody(WorldId, bodyDef);

        Polygon box = Geometry.MakeBox(groundSize, 1.0f);
        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        Shape.CreatePolygonShape(groundId, shapeDef, box);

        for (int i = 0; i < e_maxBodyCount; ++i)
        {
            m_bodies[i] = BodyId.NullId;
        }

        m_baseCount = Core.B2Debug ? 40 : 100;
        m_iterations = Core.B2Debug ? 1 : 41;
        m_bodyCount = 0;
        m_awake = false;

        m_wakeTotal = 0.0f;
        m_wakeCount = 0;

        m_sleepTotal = 0.0f;
        m_sleepCount = 0;

        CreateScene();
    }

    void CreateScene()
    {
        for (int i = 0; i < e_maxBodyCount; ++i)
        {
            if (m_bodies[i].IsNotNull)
            {
                Body.DestroyBody(m_bodies[i]);
                m_bodies[i] = BodyId.NullId;
            }
        }

        int count = m_baseCount;
        float rad = 0.5f;
        float shift = rad * 2.0f;
        float centerx = shift * count / 2.0f;
        float centery = shift / 2.0f + 1.0f;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;
        shapeDef.Friction = 0.5f;

        float h = 0.5f;
        Polygon box = Geometry.MakeRoundedBox(h, h, 0.0f);

        int index = 0;

        for (int i = 0; i < count; ++i)
        {
            float y = i * shift + centery;

            for (int j = i; j < count; ++j)
            {
                float x = 0.5f * i * shift + (j - i) * shift - centerx;
                bodyDef.Position = (x, y);

                Debug.Assert(index < e_maxBodyCount);
                m_bodies[index] = Body.CreateBody(WorldId, bodyDef);
                Shape.CreatePolygonShape(m_bodies[index], shapeDef, box);

                index += 1;
            }
        }

        m_bodyCount = index;
    }

    public override void PreStep()
    {
        var timer = Stopwatch.StartNew();

        for (int i = 0; i < m_iterations; ++i)
        {
            Body.SetAwake(m_bodies[0], m_awake);
            if (m_awake)
            {
                m_wakeTotal += timer.ElapsedMilliseconds;
                m_wakeCount += 1;
                timer.Reset();
            }
            else
            {
                m_sleepTotal += timer.ElapsedMilliseconds;
                m_sleepCount += 1;
                timer.Reset();
            }

            m_awake = !m_awake;
        }
    }

    protected override void OnRender()
    {
        if (m_wakeCount > 0)
        {
            DrawString($"wake ave = {m_wakeTotal / m_wakeCount} ms");
        }

        if (m_sleepCount > 0)
        {
            DrawString($"sleep ave = {m_sleepTotal / m_sleepCount} ms");
        }

        base.OnRender();
    }
}