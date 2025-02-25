using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Collision;

// Tests manifolds and contact points
[Sample("Collision", "Manifold")]
public class ManifoldSample : SampleBase
{
    protected DistanceCache m_smgroxCache1;

    protected DistanceCache m_smgroxCache2;

    protected DistanceCache m_smgcapCache1;

    protected DistanceCache m_smgcapCache2;

    protected Hull m_wedge;

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

    protected bool m_enableCaching;

    public ManifoldSample(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (1.8f, 0.0f);
            Global.Camera.Zoom = 25.0f * 0.45f;
        }

        m_smgroxCache1 = new();
        m_smgroxCache2 = new();
        m_smgcapCache1 = new();
        m_smgcapCache2 = new();

        m_transform = Transform.Identity;
        m_angle = 0.0f;
        m_round = 0.1f;

        m_startPoint = (0.0f, 0.0f);
        m_basePosition = (0.0f, 0.0f);
        m_baseAngle = 0.0f;

        m_dragging = false;
        m_rotating = false;
        m_showIds = false;
        m_showSeparation = false;
        m_showAnchors = false;
        m_enableCaching = true;

        Vec2[] points = [(-0.1f, -0.5f), (0.1f, -0.5f), (0.0f, 0.5f)];
        m_wedge = HullFunc.ComputeHull(points, 3);
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
            m_transform.P.X = m_basePosition.X + 0.5f * (p.X - m_startPoint.X);
            m_transform.P.Y = m_basePosition.Y + 0.5f * (p.Y - m_startPoint.Y);
        }
        else if (m_rotating)
        {
            float dx = p.X - m_startPoint.X;
            m_angle = Math.Clamp(m_baseAngle + 1.0f * dx, -B2Math.Pi, B2Math.Pi);
            m_transform.Q = B2Math.MakeRot(m_angle);
        }
    }

    void DrawManifold(in Manifold manifold, Vec2 origin1, Vec2 origin2)
    {
        for (int i = 0; i < manifold.PointCount; ++i)
        {
            ref readonly ManifoldPoint mp = ref manifold.Points[i];

            Vec2 p1 = mp.Point;
            Vec2 p2 = B2Math.MulAdd(p1, 0.5f, manifold.Normal);
            Draw.DrawSegment(p1, p2, B2HexColor.White);

            if (m_showAnchors)
            {
                Draw.DrawPoint(origin1 + mp.AnchorA, 5.0f, B2HexColor.Red);
                Draw.DrawPoint(origin2 + mp.AnchorB, 5.0f, B2HexColor.Green);
            }
            else
            {
                Draw.DrawPoint(p1, 5.0f, B2HexColor.Blue);
            }

            if (m_showIds)
            {
                // uint32_t indexA = mp.id >> 8;
                // uint32_t indexB = 0xFF & mp.id;
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
        Vec2 offset = (-10.0f, -5.0f);
        Vec2 increment = (4.0f, 0.0f);

        B2HexColor color1 = B2HexColor.Aquamarine;
        B2HexColor color2 = B2HexColor.Magenta;

        if (m_enableCaching == false)
        {
            m_smgroxCache1 = new();
            m_smgroxCache2 = new();
            m_smgcapCache1 = new();
            m_smgcapCache2 = new();
        }

        // circle-circle
        {
            Circle circle1 = ((0.0f, 0.0f), 0.5f);
            Circle circle2 = ((0.0f, 0.0f), 1.0f);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            Manifold m = ManifoldFunc.CollideCircles(circle1, transform1, circle2, transform2);

            Draw.DrawSolidCircle(transform1, circle1.Center, circle1.Radius, color1);
            Draw.DrawSolidCircle(transform2, circle2.Center, circle2.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset = offset + increment;
        }

        // capsule-circle
        {
            Capsule capsule = ((-0.5f, 0.0f), (0.5f, 0.0f), 0.25f);
            Circle circle = ((0.0f, 0.0f), 0.5f);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            Manifold m = ManifoldFunc.CollideCapsuleAndCircle(capsule, transform1, circle, transform2);

            Vec2 v1 = B2Math.TransformPoint(transform1, capsule.Center1);
            Vec2 v2 = B2Math.TransformPoint(transform1, capsule.Center2);
            Draw.DrawSolidCapsule(v1, v2, capsule.Radius, color1);

            Draw.DrawSolidCircle(transform2, circle.Center, circle.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset = (offset + increment);
        }

        // segment-circle
        {
            Segment segment = ((-1.0f, 0.0f), (1.0f, 0.0f));
            Circle circle = ((0.0f, 0.0f), 0.5f);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            Manifold m = ManifoldFunc.CollideSegmentAndCircle(segment, transform1, circle, transform2);

            Vec2 p1 = B2Math.TransformPoint(transform1, segment.Point1);
            Vec2 p2 = B2Math.TransformPoint(transform1, segment.Point2);
            Draw.DrawSegment(p1, p2, color1);

            Draw.DrawSolidCircle(transform2, circle.Center, circle.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset = offset + increment;
        }

        // box-circle
        {
            Circle circle = ((0.0f, 0.0f), 0.5f);
            Polygon box = Geometry.MakeSquare(0.5f);
            box.Radius = m_round;

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            Manifold m = ManifoldFunc.CollidePolygonAndCircle(box, transform1, circle, transform2);

            Draw.DrawSolidPolygon(transform1, box.Vertices, box.Count, m_round, color1);
            Draw.DrawSolidCircle(transform2, circle.Center, circle.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset = offset + increment;
        }

        // capsule-capsule
        {
            Capsule capsule = ((-0.5f, 0.0f), (0.5f, 0.0f), 0.25f);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            Manifold m = ManifoldFunc.CollideCapsules(capsule, transform1, capsule, transform2);

            Vec2 v1 = B2Math.TransformPoint(transform1, capsule.Center1);
            Vec2 v2 = B2Math.TransformPoint(transform1, capsule.Center2);
            Draw.DrawSolidCapsule(v1, v2, capsule.Radius, color1);

            v1 = B2Math.TransformPoint(transform2, capsule.Center1);
            v2 = B2Math.TransformPoint(transform2, capsule.Center2);
            Draw.DrawSolidCapsule(v1, v2, capsule.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset = (offset + increment);
        }

        // box-capsule
        {
            Capsule capsule = ((-0.4f, 0.0f), (-0.1f, 0.0f), 0.1f);
            Polygon box = Geometry.MakeOffsetBox(0.25f, 1.0f, (1.0f, -1.0f), B2Math.MakeRot(0.25f * B2Math.Pi));

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            Manifold m = ManifoldFunc.CollidePolygonAndCapsule(box, transform1, capsule, transform2);

            Draw.DrawSolidPolygon(transform1, box.Vertices, box.Count, box.Radius, color1);

            Vec2 v1 = B2Math.TransformPoint(transform2, capsule.Center1);
            Vec2 v2 = B2Math.TransformPoint(transform2, capsule.Center2);
            Draw.DrawSolidCapsule(v1, v2, capsule.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset += increment;
        }

        // segment-capsule
        {
            Segment segment = ((-1.0f, 0.0f), (1.0f, 0.0f));
            Capsule capsule = ((-0.5f, 0.0f), (0.5f, 0.0f), 0.25f);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            Manifold m = ManifoldFunc.CollideSegmentAndCapsule(segment, transform1, capsule, transform2);

            Vec2 p1 = B2Math.TransformPoint(transform1, segment.Point1);
            Vec2 p2 = B2Math.TransformPoint(transform1, segment.Point2);
            Draw.DrawSegment(p1, p2, color1);

            p1 = B2Math.TransformPoint(transform2, capsule.Center1);
            p2 = B2Math.TransformPoint(transform2, capsule.Center2);
            Draw.DrawSolidCapsule(p1, p2, capsule.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset += increment;
        }

        offset = (-10.0f, 0.0f);

        // box-box
        {
            Polygon box1 = Geometry.MakeBox(2.0f, 0.1f);
            Polygon box = Geometry.MakeSquare(0.25f);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            // Transform transform2 = {Add({0.0f, -0.1f}, offset), {0.0f, 1.0f}};

            Manifold m = ManifoldFunc.CollidePolygons(box1, transform1, box, transform2);

            Draw.DrawSolidPolygon(transform1, box1.Vertices, box1.Count, box1.Radius, color1);
            Draw.DrawSolidPolygon(transform2, box.Vertices, box.Count, box.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset += increment;
        }

        // box-rox
        {
            Polygon box = Geometry.MakeSquare(0.5f);
            float h = 0.5f - m_round;
            Polygon rox = Geometry.MakeRoundedBox(h, h, m_round);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            // Transform transform2 = {Add({0.0f, -0.1f}, offset), {0.0f, 1.0f}};

            Manifold m = ManifoldFunc.CollidePolygons(box, transform1, rox, transform2);

            Draw.DrawSolidPolygon(transform1, box.Vertices, box.Count, box.Radius, color1);
            Draw.DrawSolidPolygon(transform2, rox.Vertices, rox.Count, rox.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset += increment;
        }

        // rox-rox
        {
            float h = 0.5f - m_round;
            Polygon rox = Geometry.MakeRoundedBox(h, h, m_round);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            // Transform transform1 = {{6.48024225f, 2.07872653f}, {-0.938356698f, 0.345668465f}};
            // Transform transform2 = {{5.52862263f, 2.51146317f}, {-0.859374702f, -0.511346340f}};

            Manifold m = ManifoldFunc.CollidePolygons(rox, transform1, rox, transform2);

            Draw.DrawSolidPolygon(transform1, rox.Vertices, rox.Count, rox.Radius, color1);
            Draw.DrawSolidPolygon(transform2, rox.Vertices, rox.Count, rox.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset = offset + increment;
        }

        // segment-rox
        {
            Segment segment = ((-1.0f, 0.0f), (1.0f, 0.0f));
            float h = 0.5f - m_round;
            Polygon rox = Geometry.MakeRoundedBox(h, h, m_round);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            // Transform transform2 = {Add({-1.44583416f, 0.397352695f}, offset), m_transform.Q};

            Manifold m = ManifoldFunc.CollideSegmentAndPolygon(segment, transform1, rox, transform2);

            Vec2 p1 = B2Math.TransformPoint(transform1, segment.Point1);
            Vec2 p2 = B2Math.TransformPoint(transform1, segment.Point2);
            Draw.DrawSegment(p1, p2, color1);
            Draw.DrawSolidPolygon(transform2, rox.Vertices, rox.Count, rox.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset = offset + increment;
        }

        // wox-wox
        {
            Polygon wox = Geometry.MakePolygon(m_wedge, m_round);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            // Transform transform2 = {Add({0.0f, -0.1f}, offset), {0.0f, 1.0f}};

            Manifold m = ManifoldFunc.CollidePolygons(wox, transform1, wox, transform2);

            Draw.DrawSolidPolygon(transform1, wox.Vertices, wox.Count, wox.Radius, color1);
            Draw.DrawSolidPolygon(transform1, wox.Vertices, wox.Count, 0.0f, color1);
            Draw.DrawSolidPolygon(transform2, wox.Vertices, wox.Count, wox.Radius, color2);
            Draw.DrawSolidPolygon(transform2, wox.Vertices, wox.Count, 0.0f, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset = offset + increment;
        }

        // wox-wox
        {
            Vec2[] p1s = [(0.175740838f, 0.224936664f), (-0.301293969f, 0.194021404f), (-0.105151534f, -0.432157338f)];
            Vec2[] p2s = [(-0.427884758f, -0.225028217f), (0.0566576123f, -0.128772855f), (0.176625848f, 0.338923335f)];

            Hull h1 = HullFunc.ComputeHull(p1s, 3);
            Hull h2 = HullFunc.ComputeHull(p2s, 3);
            Polygon w1 = Geometry.MakePolygon(h1, 0.158798501f);
            Polygon w2 = Geometry.MakePolygon(h2, 0.205900759f);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            // Transform transform2 = {Add({0.0f, -0.1f}, offset), {0.0f, 1.0f}};

            Manifold m = ManifoldFunc.CollidePolygons(w1, transform1, w2, transform2);

            Draw.DrawSolidPolygon(transform1, w1.Vertices, w1.Count, w1.Radius, color1);
            Draw.DrawSolidPolygon(transform1, w1.Vertices, w1.Count, 0.0f, color1);
            Draw.DrawSolidPolygon(transform2, w2.Vertices, w2.Count, w2.Radius, color2);
            Draw.DrawSolidPolygon(transform2, w2.Vertices, w2.Count, 0.0f, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset = offset + increment;
        }

        offset = (-10.0f, 5.0f);

        // chain-segment vs circle
        {
            ChainSegment segment = ((2.0f, 1.0f), ((1.0f, 1.0f), (-1.0f, 0.0f)), (-2.0f, 0.0f), -1);
            Circle circle = ((0.0f, 0.0f), 0.5f);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            Manifold m = ManifoldFunc.CollideChainSegmentAndCircle(segment, transform1, circle, transform2);

            Vec2 g1 = B2Math.TransformPoint(transform1, segment.Ghost1);
            Vec2 g2 = B2Math.TransformPoint(transform1, segment.Ghost2);
            Vec2 p1 = B2Math.TransformPoint(transform1, segment.Segment.Point1);
            Vec2 p2 = B2Math.TransformPoint(transform1, segment.Segment.Point2);
            Draw.DrawSegment(g1, p1, B2HexColor.LightGray);
            Draw.DrawSegment(p1, p2, color1);
            Draw.DrawSegment(p2, g2, B2HexColor.LightGray);
            Draw.DrawSolidCircle(transform2, circle.Center, circle.Radius, color2);

            DrawManifold(m, transform1.P, transform2.P);

            offset.X += 2.0f * increment.X;
        }

        // chain-segment vs rounded polygon
        {
            ChainSegment segment1 = ((2.0f, 1.0f), ((1.0f, 1.0f), (-1.0f, 0.0f)), (-2.0f, 0.0f), -1);
            ChainSegment segment2 = ((3.0f, 1.0f), ((2.0f, 1.0f), (1.0f, 1.0f)), (-1.0f, 0.0f), -1);

            // ChainSegment segment1 = {{2.0f, 0.0f}, {{1.0f, 0.0f}, {-1.0f, 0.0f}}, {-2.0f, 0.0f}, -1};
            // ChainSegment segment2 = {{3.0f, 0.0f}, {{2.0f, 0.0f}, {1.0f, 0.0f}}, {-1.0f, 0.0f}, -1};
            // ChainSegment segment1 = {{0.5f, 1.0f}, {{0.0f, 2.0f}, {-0.5f, 1.0f}}, {-1.0f, 0.0f}, -1};
            // ChainSegment segment2 = {{1.0f, 0.0f}, {{0.5f, 1.0f}, {0.0f, 2.0f}}, {-0.5f, 1.0f}, -1};
            float h = 0.5f - m_round;
            Polygon rox = Geometry.MakeRoundedBox(h, h, m_round);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            Manifold m1 = ManifoldFunc.CollideChainSegmentAndPolygon(segment1, transform1, rox, transform2, ref m_smgroxCache1);
            Manifold m2 = ManifoldFunc.CollideChainSegmentAndPolygon(segment2, transform1, rox, transform2, ref m_smgroxCache2);

            {
                Vec2 g1 = B2Math.TransformPoint(transform1, segment1.Ghost1);
                Vec2 g2 = B2Math.TransformPoint(transform1, segment1.Ghost2);
                Vec2 p1 = B2Math.TransformPoint(transform1, segment1.Segment.Point1);
                Vec2 p2 = B2Math.TransformPoint(transform1, segment1.Segment.Point2);
                Draw.DrawSegment(p1, p2, color1);
                Draw.DrawPoint(p1, 4.0f, color1);
                Draw.DrawPoint(p2, 4.0f, color1);
                Draw.DrawSegment(p2, g2, B2HexColor.LightGray);
            }

            {
                Vec2 g1 = B2Math.TransformPoint(transform1, segment2.Ghost1);
                Vec2 g2 = B2Math.TransformPoint(transform1, segment2.Ghost2);
                Vec2 p1 = B2Math.TransformPoint(transform1, segment2.Segment.Point1);
                Vec2 p2 = B2Math.TransformPoint(transform1, segment2.Segment.Point2);
                Draw.DrawSegment(g1, p1, B2HexColor.LightGray);
                Draw.DrawSegment(p1, p2, color1);
                Draw.DrawPoint(p1, 4.0f, color1);
                Draw.DrawPoint(p2, 4.0f, color1);
            }

            Draw.DrawSolidPolygon(transform2, rox.Vertices, rox.Count, rox.Radius, color2);
            Draw.DrawPoint(B2Math.TransformPoint(transform2, rox.Centroid), 5.0f, B2HexColor.Gainsboro);

            DrawManifold(m1, transform1.P, transform2.P);
            DrawManifold(m2, transform1.P, transform2.P);

            offset.X += 2.0f * increment.X;
        }

        // chain-segment vs capsule
        {
            ChainSegment segment1 = ((2.0f, 1.0f), ((1.0f, 1.0f), (-1.0f, 0.0f)), (-2.0f, 0.0f), -1);
            ChainSegment segment2 = ((3.0f, 1.0f), ((2.0f, 1.0f), (1.0f, 1.0f)), (-1.0f, 0.0f), -1);
            Capsule capsule = ((-0.5f, 0.0f), (0.5f, 0.0f), 0.25f);

            Transform transform1 = (offset, Rot.Identity);
            Transform transform2 = (m_transform.P + offset, m_transform.Q);

            Manifold m1 = ManifoldFunc.CollideChainSegmentAndCapsule(segment1, transform1, capsule, transform2, ref m_smgcapCache1);
            Manifold m2 = ManifoldFunc.CollideChainSegmentAndCapsule(segment2, transform1, capsule, transform2, ref m_smgcapCache2);

            {
                Vec2 g1 = B2Math.TransformPoint(transform1, segment1.Ghost1);
                Vec2 g2 = B2Math.TransformPoint(transform1, segment1.Ghost2);
                Vec2 p1 = B2Math.TransformPoint(transform1, segment1.Segment.Point1);
                Vec2 p2 = B2Math.TransformPoint(transform1, segment1.Segment.Point2);

                // Draw.DrawSegment(g1, p1, B2HexColor.LightGray);
                Draw.DrawSegment(p1, p2, color1);
                Draw.DrawPoint(p1, 4.0f, color1);
                Draw.DrawPoint(p2, 4.0f, color1);
                Draw.DrawSegment(p2, g2, B2HexColor.LightGray);
            }

            {
                Vec2 g1 = B2Math.TransformPoint(transform1, segment2.Ghost1);
                Vec2 g2 = B2Math.TransformPoint(transform1, segment2.Ghost2);
                Vec2 p1 = B2Math.TransformPoint(transform1, segment2.Segment.Point1);
                Vec2 p2 = B2Math.TransformPoint(transform1, segment2.Segment.Point2);
                Draw.DrawSegment(g1, p1, B2HexColor.LightGray);
                Draw.DrawSegment(p1, p2, color1);
                Draw.DrawPoint(p1, 4.0f, color1);
                Draw.DrawPoint(p2, 4.0f, color1);

                // Draw.DrawSegment(p2, g2, B2HexColor.LightGray);
            }
            {
                Vec2 p1 = B2Math.TransformPoint(transform2, capsule.Center1);
                Vec2 p2 = B2Math.TransformPoint(transform2, capsule.Center2);
                Draw.DrawSolidCapsule(p1, p2, capsule.Radius, color2);

                Draw.DrawPoint(B2Math.Lerp(p1, p2, 0.5f), 5.0f, B2HexColor.Gainsboro);
            }
            DrawManifold(m1, transform1.P, transform2.P);
            DrawManifold(m2, transform1.P, transform2.P);

            offset.X += 2.0f * increment.X;
        }
    }
}