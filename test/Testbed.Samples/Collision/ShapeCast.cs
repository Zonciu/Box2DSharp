using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Collision;

[Sample("Collision", "Shape Cast")]
public class ShapeCast : SampleBase
{
    Vec2[] m_vAs = new Vec2[Core.MaxPolygonVertices];

    int m_countA;

    float m_radiusA;

    Vec2[] m_vBs = new Vec2[Core.MaxPolygonVertices];

    int m_countB;

    float m_radiusB;

    Transform m_transformA;

    Transform m_transformB;

    Vec2 m_translationB;

    bool m_rayDrag;

    public ShapeCast(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (-1.5f, 1.0f);
            Global.Camera.Zoom = 25.0f * 0.2f;
        }

        // box swept against a segment
        m_vAs[0] = (-2.0f, 0.0f);
        m_vAs[1] = (2.0f, 0.0f);
        m_countA = 2;
        m_radiusA = 0.0f;

        m_vBs[0] = (-0.25f, -0.25f);
        m_vBs[1] = (0.25f, -0.25f);
        m_vBs[2] = (0.25f, 0.25f);
        m_vBs[3] = (-0.25f, 0.25f);
        m_countB = 4;
        m_radiusB = 0.25f;

        m_transformA.P = (0.0f, 0.0f);
        m_transformA.Q = B2Math.MakeRot(0.25f * B2Math.Pi);
        m_transformB.P = (-8.0f, 0.0f);
        m_transformB.Q = Rot.Identity;
        m_translationB = (8.0f, 0.0f);

        m_rayDrag = false;
    }

    public override void MouseDown(Vec2 p, MouseInputEventArgs e)
    {
        base.MouseDown(p, e);

        if (e.Button == MouseButton.Left)
        {
            m_transformB.P = p;
            m_rayDrag = true;
        }
    }

    public override void MouseUp(Vec2 p, MouseInputEventArgs e)
    {
        base.MouseUp(p, e);

        if (e.Button == MouseButton.Left)
        {
            m_rayDrag = false;
        }
    }

    public override void MouseMove(Vec2 p, MouseMoveEventArgs e)
    {
        base.MouseMove(p, e);

        if (m_rayDrag)
        {
            m_translationB = p - m_transformB.P;
        }
    }

    protected override void OnRender()
    {
        base.OnRender();

        ShapeCastPairInput input = new();
        input.ProxyA = DistanceFunc.MakeProxy(m_vAs, m_countA, m_radiusA);
        input.ProxyB = DistanceFunc.MakeProxy(m_vBs, m_countB, m_radiusB);
        input.TransformA = m_transformA;
        input.TransformB = m_transformB;
        input.TranslationB = m_translationB;
        input.MaxFraction = 1.0f;

        CastOutput output = DistanceFunc.ShapeCast(input);

        Transform transform;
        transform.Q = m_transformB.Q;
        transform.P = B2Math.MulAdd(m_transformB.P, output.Fraction, input.TranslationB);

        DistanceInput distanceInput;
        distanceInput.ProxyA = DistanceFunc.MakeProxy(m_vAs, m_countA, m_radiusA);
        distanceInput.ProxyB = DistanceFunc.MakeProxy(m_vBs, m_countB, m_radiusB);
        distanceInput.TransformA = m_transformA;
        distanceInput.TransformB = transform;
        distanceInput.UseRadii = false;
        DistanceCache distanceCache = new();
        DistanceOutput distanceOutput = DistanceFunc.ShapeDistance(ref distanceCache, distanceInput, null, 0);

        DrawString($"hit = {(output.Hit ? "true" : "false")}, iters = {output.Iterations}, lambda = {output.Fraction}, distance = {distanceOutput.Distance}");

        Vec2[] vertices = new Vec2[Core.MaxPolygonVertices];

        for (int i = 0; i < m_countA; ++i)
        {
            vertices[i] = B2Math.TransformPoint(m_transformA, m_vAs[i]);
        }

        if (m_countA == 1)
        {
            if (m_radiusA > 0.0f)
            {
                Draw.DrawCircle(vertices[0], m_radiusA, B2HexColor.Gray9);
            }
            else
            {
                Draw.DrawPoint(vertices[0], 5.0f, B2HexColor.Gray9);
            }
        }
        else
        {
            Draw.DrawSolidPolygon(Transform.Identity, vertices, m_countA, m_radiusA, B2HexColor.Gray9);
        }

        for (int i = 0; i < m_countB; ++i)
        {
            vertices[i] = B2Math.TransformPoint(m_transformB, m_vBs[i]);
        }

        if (m_countB == 1)
        {
            if (m_radiusB > 0.0f)
            {
                Draw.DrawCircle(vertices[0], m_radiusB, B2HexColor.Green);
            }
            else
            {
                Draw.DrawPoint(vertices[0], 5.0f, B2HexColor.Green);
            }
        }
        else
        {
            Draw.DrawSolidPolygon(Transform.Identity, vertices, m_countB, m_radiusB, B2HexColor.Green);
        }

        for (int i = 0; i < m_countB; ++i)
        {
            vertices[i] = B2Math.TransformPoint(transform, m_vBs[i]);
        }

        if (m_countB == 1)
        {
            if (m_radiusB > 0.0f)
            {
                Draw.DrawCircle(vertices[0], m_radiusB, B2HexColor.Orange);
            }
            else
            {
                Draw.DrawPoint(vertices[0], 5.0f, B2HexColor.Orange);
            }
        }
        else
        {
            Draw.DrawSolidPolygon(Transform.Identity, vertices, m_countB, m_radiusB, B2HexColor.Orange);
        }

        if (output.Hit)
        {
            Vec2 p1 = output.Point;
            Draw.DrawPoint(p1, 10.0f, B2HexColor.Red);
            Vec2 p2 = B2Math.MulAdd(p1, 1.0f, output.Normal);
            Draw.DrawSegment(p1, p2, B2HexColor.Red);
        }

        Draw.DrawSegment(m_transformB.P, m_transformB.P + m_translationB, B2HexColor.Gray);
    }
}