using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Collision;

[Sample("Collision", "Smooth Manifold")]
public class SmoothManifold : SampleBase
{
    protected int m_shapeType;

    protected ChainSegment[] m_segments;

    protected int m_count;

    protected Transform m_transform;

    protected float m_angle;

    protected float m_round;

    protected Vec2 m_basePosition;

    protected Vec2 m_startPoint;

    protected float m_baseAngle;

    protected bool m_dragging;

    protected bool m_rotating;

    protected bool m_showIds;

    protected bool m_showAnchors;

    protected bool m_showSeparation;

    protected static class TestShapeType
    {
        public const int e_circleShape = 0;

        public const int e_boxShape = 1;
    };

    public SmoothManifold(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (2.0f, 20.0f);
            Global.Camera.Zoom = 21.0f;
        }

        m_shapeType = TestShapeType.e_boxShape;
        m_transform = ((0.0f, 20.0f), Rot.Identity);
        m_angle = 0.0f;
        m_round = 0.0f;

        m_startPoint = (0.0f, 00.0f);
        m_basePosition = (0.0f, 0.0f);
        m_baseAngle = 0.0f;

        m_dragging = false;
        m_rotating = false;
        m_showIds = false;
        m_showAnchors = false;
        m_showSeparation = false;

        // https://betravis.github.Io/shape-tools/path-to-polygon/
        m_count = 36;

        Vec2[] points = new Vec2[36];
        points[0] = (-20.58325f, 14.54175f);
        points[1] = (-21.90625f, 15.8645f);
        points[2] = (-24.552f, 17.1875f);
        points[3] = (-27.198f, 11.89575f);
        points[4] = (-29.84375f, 15.8645f);
        points[5] = (-29.84375f, 21.15625f);
        points[6] = (-25.875f, 23.802f);
        points[7] = (-20.58325f, 25.125f);
        points[8] = (-25.875f, 29.09375f);
        points[9] = (-20.58325f, 31.7395f);
        points[10] = (-11.0089998f, 23.2290001f);
        points[11] = (-8.67700005f, 21.15625f);
        points[12] = (-6.03125f, 21.15625f);
        points[13] = (-7.35424995f, 29.09375f);
        points[14] = (-3.38549995f, 29.09375f);
        points[15] = (1.90625f, 30.41675f);
        points[16] = (5.875f, 17.1875f);
        points[17] = (11.16675f, 25.125f);
        points[18] = (9.84375f, 29.09375f);
        points[19] = (13.8125f, 31.7395f);
        points[20] = (21.75f, 30.41675f);
        points[21] = (28.3644981f, 26.448f);
        points[22] = (25.71875f, 18.5105f);
        points[23] = (24.3957481f, 13.21875f);
        points[24] = (17.78125f, 11.89575f);
        points[25] = (15.1355f, 7.92700005f);
        points[26] = (5.875f, 9.25f);
        points[27] = (1.90625f, 11.89575f);
        points[28] = (-3.25f, 11.89575f);
        points[29] = (-3.25f, 9.9375f);
        points[30] = (-4.70825005f, 9.25f);
        points[31] = (-8.67700005f, 9.25f);
        points[32] = (-11.323f, 11.89575f);
        points[33] = (-13.96875f, 11.89575f);
        points[34] = (-15.29175f, 14.54175f);
        points[35] = (-19.2605f, 14.54175f);

        m_segments = new ChainSegment[m_count];

        for (int i = 0; i < m_count; ++i)
        {
            int i0 = i > 0 ? i - 1 : m_count - 1;
            int i1 = i;
            int i2 = i1 < m_count - 1 ? i1 + 1 : 0;
            int i3 = i2 < m_count - 1 ? i2 + 1 : 0;

            Vec2 g1 = points[i0];
            Vec2 p1 = points[i1];
            Vec2 p2 = points[i2];
            Vec2 g2 = points[i3];

            m_segments[i] = (g1, (p1, p2), g2, -1);
        }
    }


    public override void MouseDown(Vec2 p, MouseInputEventArgs e)
    {
        base.MouseDown(p, e);
        if (e.Button == MouseButton.Left)
        {
            if (!e.Modifiers.IsSet(KeyModifiers.Shift) && m_rotating == false)
            {
                m_dragging = true;
                m_startPoint = p;
                m_basePosition = m_transform.P;
            }
            else if (e.Modifiers.IsSet(KeyModifiers.Shift) && m_dragging == false)
            {
                m_rotating = true;
                m_startPoint = p;
                m_baseAngle = m_angle;
            }
        }
    }

    public override void MouseUp(Vec2 p, MouseInputEventArgs e)
    {
        base.MouseUp(p, e);

        if (e.Button == MouseButton.Left)
        {
            m_dragging = false;
            m_rotating = false;
        }
    }

    public override void MouseMove(Vec2 p, MouseMoveEventArgs e)
    {
        base.MouseMove(p, e);
        if (m_dragging)
        {
            m_transform.P.X = m_basePosition.X + (p.X - m_startPoint.X);
            m_transform.P.Y = m_basePosition.Y + (p.Y - m_startPoint.Y);
        }
        else if (m_rotating)
        {
            float dx = p.X - m_startPoint.X;
            m_angle = Math.Clamp(m_baseAngle + 1.0f * dx, -B2Math.Pi, B2Math.Pi);
            m_transform.Q = B2Math.MakeRot(m_angle);
        }
    }

    void DrawManifold(in Manifold manifold)
    {
        for (int i = 0; i < manifold.PointCount; ++i)
        {
            ref readonly ManifoldPoint mp = ref manifold.Points[i];

            Vec2 p1 = mp.Point;
            Vec2 p2 = B2Math.MulAdd(p1, 0.5f, manifold.Normal);
            Draw.DrawSegment(p1, p2, B2HexColor.White);

            if (m_showAnchors)
            {
                Draw.DrawPoint(p1, 5.0f, B2HexColor.Green);
            }
            else
            {
                Draw.DrawPoint(p1, 5.0f, B2HexColor.Green);
            }

            if (m_showIds)
            {
                // uint32_t indexA = mp.Id >> 8;
                // uint32_t indexB = 0xFF & mp.Id;
                Vec2 p = (p1.X + 0.05f, p1.Y - 0.02f);
                Draw.DrawString(p, $"0x{mp.Id:X4}");
            }

            if (m_showSeparation)
            {
                Vec2 p = (p1.X + 0.05f, p1.Y + 0.03f);
                Draw.DrawString(p, mp.Separation.ToString("F3"));
            }
        }
    }

    protected override void OnRender()
    {
        base.OnRender();

        B2HexColor color1 = B2HexColor.Yellow;
        B2HexColor color2 = B2HexColor.Magenta;

        Transform transform1 = Transform.Identity;
        Transform transform2 = m_transform;

        for (int i = 0; i < m_count; ++i)
        {
            ref readonly ChainSegment segment = ref m_segments[i];
            Vec2 p1 = B2Math.TransformPoint(transform1, segment.Segment.Point1);
            Vec2 p2 = B2Math.TransformPoint(transform1, segment.Segment.Point2);
            Draw.DrawSegment(p1, p2, color1);
            Draw.DrawPoint(p1, 4.0f, color1);
        }

        // chain-segment vs circle
        if (m_shapeType == TestShapeType.e_circleShape)
        {
            Circle circle = ((0.0f, 0.0f), 0.5f);
            Draw.DrawSolidCircle(transform2, circle.Center, circle.Radius, color2);

            for (int i = 0; i < m_count; ++i)
            {
                ref readonly ChainSegment segment = ref m_segments[i];
                Manifold m = ManifoldFunc.CollideChainSegmentAndCircle(segment, transform1, circle, transform2);
                DrawManifold(m);
            }
        }
        else if (m_shapeType == TestShapeType.e_boxShape)
        {
            float h = 0.5f - m_round;
            Polygon rox = Geometry.MakeRoundedBox(h, h, m_round);
            Draw.DrawSolidPolygon(transform2, rox.Vertices, rox.Count, rox.Radius, color2);

            for (int i = 0; i < m_count; ++i)
            {
                ref readonly ChainSegment segment = ref m_segments[i];
                DistanceCache cache = new();
                Manifold m = ManifoldFunc.CollideChainSegmentAndPolygon(segment, transform1, rox, transform2, ref cache);
                DrawManifold(m);
            }
        }
    }
}