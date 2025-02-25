using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Collision;

[Sample("Collision", "Ray Cast")]
public class RayCast : SampleBase
{
    protected Polygon Box;

    protected Polygon Triangle;

    protected Circle Circle;

    protected Capsule Capsule;

    protected Segment Segment;

    protected Transform Transform;

    protected float Angle;

    protected Vec2 RayStart;

    protected Vec2 RayEnd;

    protected Vec2 BasePosition;

    protected float BaseAngle;

    protected Vec2 StartPosition;

    protected bool RayDrag;

    protected bool Translating;

    protected bool Rotating;

    protected bool ShowFraction;

    public RayCast(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 20.0f);
            Global.Camera.Zoom = 17.5f;
        }

        Circle = ((0.0f, 0.0f), 2.0f);
        Capsule = ((-1.0f, 1.0f), (1.0f, -1.0f), 1.5f);
        Box = Geometry.MakeBox(2.0f, 2.0f);

        Vec2[] vertices = [(-2.0f, 0.0f), (2.0f, 0.0f), (2.0f, 3.0f)];
        Hull hull = HullFunc.ComputeHull(vertices, 3);
        Triangle = Geometry.MakePolygon(hull, 0.0f);

        Segment = ((-3.0f, 0.0f), (3.0f, 0.0f));

        Transform = Transform.Identity;
        Angle = 0.0f;

        BasePosition = (0.0f, 0.0f);
        BaseAngle = 0.0f;
        StartPosition = (0.0f, 0.0f);

        RayStart = (0.0f, 30.0f);
        RayEnd = (0.0f, 0.0f);

        RayDrag = false;
        Translating = false;
        Rotating = false;

        ShowFraction = false;
    }

    public override void MouseDown(Vec2 p, MouseInputEventArgs e)
    {
        base.MouseDown(p, e);
        if (e.Button == MouseButton.Left)
        {
            StartPosition = p;

            if (!e.Modifiers.IsSet(KeyModifiers.Shift) && !e.Modifiers.IsSet(KeyModifiers.Control))
            {
                RayStart = p;
                RayDrag = true;
            }
            else if (e.Modifiers.IsSet(KeyModifiers.Shift))
            {
                Translating = true;
                BasePosition = Transform.P;
            }
            else if (e.Modifiers.IsSet(KeyModifiers.Control))
            {
                Rotating = true;
                BaseAngle = Angle;
            }
        }
    }

    public override void MouseUp(Vec2 p, MouseInputEventArgs e)
    {
        base.MouseUp(p, e);
        if (e.Button == MouseButton.Left)
        {
            RayDrag = false;
            Rotating = false;
            Translating = false;
        }
    }

    public override void MouseMove(Vec2 p, MouseMoveEventArgs e)
    {
        base.MouseMove(p, e);
        if (RayDrag)
        {
            RayEnd = p;
        }
        else if (Translating)
        {
            Transform.P.X = BasePosition.X + 0.5f * (p.X - StartPosition.X);
            Transform.P.Y = BasePosition.Y + 0.5f * (p.Y - StartPosition.Y);
        }
        else if (Rotating)
        {
            float dx = p.X - StartPosition.X;
            Angle = Math.Clamp(BaseAngle + 0.5f * dx, -B2Math.Pi, B2Math.Pi);
            Transform.Q = B2Math.MakeRot(Angle);
        }
    }

    protected void DrawRay(in CastOutput output)
    {
        Vec2 p1 = RayStart;
        Vec2 p2 = RayEnd;
        Vec2 d = p2 - p1;

        if (output.Hit)
        {
            Vec2 p = B2Math.MulAdd(p1, output.Fraction, d);
            Draw.DrawSegment(p1, p, B2HexColor.White);
            Draw.DrawPoint(p1, 5.0f, B2HexColor.Green);
            Draw.DrawPoint(output.Point, 5.0f, B2HexColor.White);

            Vec2 n = B2Math.MulAdd(p, 1.0f, output.Normal);
            Draw.DrawSegment(p, n, B2HexColor.Violet);

            // if (m_rayRadius > 0.0f)
            //{
            //	Draw.DrawCircle(p1, m_rayRadius, B2HexColor.Green);
            //	Draw.DrawCircle(p, m_rayRadius, B2HexColor.Red);
            // }

            if (ShowFraction)
            {
                Vec2 ps = (p.X + 0.05f, p.Y - 0.02f);
                Draw.DrawString(ps, $"{output.Fraction:F2}");
            }
        }
        else
        {
            Draw.DrawSegment(p1, p2, B2HexColor.White);
            Draw.DrawPoint(p1, 5.0f, B2HexColor.Green);
            Draw.DrawPoint(p2, 5.0f, B2HexColor.Red);

            // if (m_rayRadius > 0.0f)
            //{
            //	Draw.DrawCircle(p1, m_rayRadius, B2HexColor.Green);
            //	Draw.DrawCircle(p2, m_rayRadius, B2HexColor.Red);
            // }
        }
    }

    protected override void OnRender()
    {
        Vec2 offset = (-20.0f, 20.0f);
        Vec2 increment = (10.0f, 0.0f);

        B2HexColor color1 = B2HexColor.Yellow;

        CastOutput output = new();
        float maxFraction = 1.0f;

        // circle
        {
            Transform transform = (Transform.P + offset, Transform.Q);
            Draw.DrawSolidCircle(transform, Circle.Center, Circle.Radius, color1);

            Vec2 start = B2Math.InvTransformPoint(transform, RayStart);
            Vec2 translation = B2Math.InvRotateVector(transform.Q, RayEnd - RayStart);
            RayCastInput input = (start, translation, maxFraction);

            CastOutput localOutput = Geometry.RayCastCircle(input, Circle);
            if (localOutput.Hit)
            {
                output = localOutput;
                output.Point = B2Math.TransformPoint(transform, localOutput.Point);
                output.Normal = B2Math.RotateVector(transform.Q, localOutput.Normal);
                maxFraction = localOutput.Fraction;
            }

            offset += increment;
        }

        // capsule
        {
            Transform transform = (Transform.P + offset, Transform.Q);
            Vec2 v1 = B2Math.TransformPoint(transform, Capsule.Center1);
            Vec2 v2 = B2Math.TransformPoint(transform, Capsule.Center2);
            Draw.DrawSolidCapsule(v1, v2, Capsule.Radius, color1);

            Vec2 start = B2Math.InvTransformPoint(transform, RayStart);
            Vec2 translation = B2Math.InvRotateVector(transform.Q, RayEnd - RayStart);
            RayCastInput input = (start, translation, maxFraction);

            CastOutput localOutput = Geometry.RayCastCapsule(input, Capsule);
            if (localOutput.Hit)
            {
                output = localOutput;
                output.Point = B2Math.TransformPoint(transform, localOutput.Point);
                output.Normal = B2Math.RotateVector(transform.Q, localOutput.Normal);
                maxFraction = localOutput.Fraction;
            }

            offset += increment;
        }

        // box
        {
            Transform transform = ((Transform.P + offset), Transform.Q);
            Draw.DrawSolidPolygon(transform, Box.Vertices, Box.Count, 0.0f, color1);

            Vec2 start = B2Math.InvTransformPoint(transform, RayStart);
            Vec2 translation = B2Math.InvRotateVector(transform.Q, RayEnd - RayStart);
            RayCastInput input = new(start, translation, maxFraction);

            CastOutput localOutput = Geometry.RayCastPolygon(input, Box);
            if (localOutput.Hit)
            {
                output = localOutput;
                output.Point = B2Math.TransformPoint(transform, localOutput.Point);
                output.Normal = B2Math.RotateVector(transform.Q, localOutput.Normal);
                maxFraction = localOutput.Fraction;
            }

            offset += increment;
        }

        // triangle
        {
            Transform transform = (Transform.P + offset, Transform.Q);
            Draw.DrawSolidPolygon(transform, Triangle.Vertices, Triangle.Count, 0.0f, color1);

            Vec2 start = B2Math.InvTransformPoint(transform, RayStart);
            Vec2 translation = B2Math.InvRotateVector(transform.Q, RayEnd - RayStart);
            RayCastInput input = new(start, translation, maxFraction);

            CastOutput localOutput = Geometry.RayCastPolygon(input, Triangle);
            if (localOutput.Hit)
            {
                output = localOutput;
                output.Point = B2Math.TransformPoint(transform, localOutput.Point);
                output.Normal = B2Math.RotateVector(transform.Q, localOutput.Normal);
                maxFraction = localOutput.Fraction;
            }

            offset += increment;
        }

        // segment
        {
            Transform transform = (Transform.P + offset, Transform.Q);

            Vec2 p1 = B2Math.TransformPoint(transform, Segment.Point1);
            Vec2 p2 = B2Math.TransformPoint(transform, Segment.Point2);
            Draw.DrawSegment(p1, p2, color1);

            Vec2 start = B2Math.InvTransformPoint(transform, RayStart);
            Vec2 translation = B2Math.InvRotateVector(transform.Q, RayEnd - RayStart);
            RayCastInput input = new(start, translation, maxFraction);

            CastOutput localOutput = Geometry.RayCastSegment(input, Segment, false);
            if (localOutput.Hit)
            {
                output = localOutput;
                output.Point = B2Math.TransformPoint(transform, localOutput.Point);
                output.Normal = B2Math.RotateVector(transform.Q, localOutput.Normal);
                maxFraction = localOutput.Fraction;
            }

            offset += increment;
        }

        DrawRay(output);
    }
}