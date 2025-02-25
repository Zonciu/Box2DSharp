using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Continuous;

// This sample shows ghost collisions
[Sample("Continuous", "Ghost Collision")]
public class GhostCollision : SampleBase
{
    BodyId m_groundId;

    BodyId m_bodyId;

    protected ShapeId m_shapeId;

    protected int m_shapeType;

    protected float m_round;

    protected float m_friction;

    protected float m_bevel;

    protected bool m_useChain;

    protected static class TestShapeType
    {
        public const int e_circleShape = 0;

        public const int e_capsuleShape = 1;

        public const int e_boxShape = 2;
    };

    public GhostCollision(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (1.5f, 16.0f);
            Global.Camera.Zoom = 25.0f * 0.8f;
        }

        m_groundId = BodyId.NullId;
        m_bodyId = BodyId.NullId;
        m_shapeId = ShapeId.NullId;
        m_shapeType = TestShapeType.e_circleShape;
        m_round = 0.0f;
        m_friction = 0.2f;
        m_bevel = 0.0f;
        m_useChain = true;

        CreateScene();
        Launch();
    }

    protected void CreateScene()
    {
        if (m_groundId.IsNotNull)
        {
            Body.DestroyBody(m_groundId);
        }

        m_shapeId = ShapeId.NullId;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        m_groundId = Body.CreateBody(WorldId, bodyDef);

        float m = 1.0f / MathF.Sqrt(2.0f);
        float mm = 2.0f * (MathF.Sqrt(2.0f) - 1.0f);
        float hx = 4.0f, hy = 0.25f;

        if (m_useChain)
        {
            Vec2[] points = new Vec2[20];
            points[0] = (-3.0f * hx, hy);
            points[1] = points[0] + (-2.0f * hx * m, 2.0f * hx * m);
            points[2] = points[1] + (-2.0f * hx * m, 2.0f * hx * m);
            points[3] = points[2] + (-2.0f * hx * m, 2.0f * hx * m);
            points[4] = points[3] + (-2.0f * hy * m, -2.0f * hy * m);
            points[5] = points[4] + (2.0f * hx * m, -2.0f * hx * m);
            points[6] = points[5] + (2.0f * hx * m, -2.0f * hx * m);
            points[7] = points[6] + (2.0f * hx * m + 2.0f * hy * (1.0f - m), -2.0f * hx * m - 2.0f * hy * (1.0f - m));
            points[8] = points[7] + (2.0f * hx + hy * mm, 0.0f);
            points[9] = points[8] + (2.0f * hx, 0.0f);
            points[10] = points[9] + (2.0f * hx + hy * mm, 0.0f);
            points[11] = points[10] + (2.0f * hx * m + 2.0f * hy * (1.0f - m), 2.0f * hx * m + 2.0f * hy * (1.0f - m));
            points[12] = points[11] + (2.0f * hx * m, 2.0f * hx * m);
            points[13] = points[12] + (2.0f * hx * m, 2.0f * hx * m);
            points[14] = points[13] + (-2.0f * hy * m, 2.0f * hy * m);
            points[15] = points[14] + (-2.0f * hx * m, -2.0f * hx * m);
            points[16] = points[15] + (-2.0f * hx * m, -2.0f * hx * m);
            points[17] = points[16] + (-2.0f * hx * m, -2.0f * hx * m);
            points[18] = points[17] + (-2.0f * hx, 0.0f);
            points[19] = points[18] + (-2.0f * hx, 0.0f);

            ChainDef chainDef = ChainDef.DefaultChainDef();
            chainDef.Points = points;
            chainDef.Count = 20;
            chainDef.IsLoop = true;
            chainDef.Friction = m_friction;

            Shape.CreateChain(m_groundId, chainDef);
        }
        else
        {
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = m_friction;

            Hull hull = new();

            if (m_bevel > 0.0f)
            {
                float hb = m_bevel;
                Vec2[] vs =
                [
                    (hx + hb, hy - 0.05f), (hx, hy), (-hx, hy), (-hx - hb, hy - 0.05f),
                    (-hx - hb, -hy + 0.05f), (-hx, -hy), (hx, -hy), (hx + hb, -hy + 0.05f)
                ];
                hull = HullFunc.ComputeHull(vs, 8);
            }
            else
            {
                Vec2[] vs = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)];
                hull = HullFunc.ComputeHull(vs, 4);
            }

            Transform transform;
            float x, y;

            // Left slope
            x = -3.0f * hx - m * hx - m * hy;
            y = hy + m * hx - m * hy;
            transform.Q = B2Math.MakeRot(-0.25f * B2Math.Pi);

            {
                transform.P = (x, y);
                Polygon polygon = Geometry.MakeOffsetPolygon(hull, 0.0f, transform);
                Shape.CreatePolygonShape(m_groundId, shapeDef, polygon);
                x -= 2.0f * m * hx;
                y += 2.0f * m * hx;
            }
            {
                transform.P = (x, y);
                Polygon polygon = Geometry.MakeOffsetPolygon(hull, 0.0f, transform);
                Shape.CreatePolygonShape(m_groundId, shapeDef, polygon);
                x -= 2.0f * m * hx;
                y += 2.0f * m * hx;
            }
            {
                transform.P = (x, y);
                Polygon polygon = Geometry.MakeOffsetPolygon(hull, 0.0f, transform);
                Shape.CreatePolygonShape(m_groundId, shapeDef, polygon);
                x -= 2.0f * m * hx;
                y += 2.0f * m * hx;
            }

            x = -2.0f * hx;
            y = 0.0f;
            transform.Q = B2Math.MakeRot(0.0f);

            {
                transform.P = (x, y);
                Polygon polygon = Geometry.MakeOffsetPolygon(hull, 0.0f, transform);
                Shape.CreatePolygonShape(m_groundId, shapeDef, polygon);
                x += 2.0f * hx;
            }
            {
                transform.P = (x, y);
                Polygon polygon = Geometry.MakeOffsetPolygon(hull, 0.0f, transform);
                Shape.CreatePolygonShape(m_groundId, shapeDef, polygon);
                x += 2.0f * hx;
            }
            {
                transform.P = (x, y);
                Polygon polygon = Geometry.MakeOffsetPolygon(hull, 0.0f, transform);
                Shape.CreatePolygonShape(m_groundId, shapeDef, polygon);
                x += 2.0f * hx;
            }

            x = 3.0f * hx + m * hx + m * hy;
            y = hy + m * hx - m * hy;
            transform.Q = B2Math.MakeRot(0.25f * B2Math.Pi);

            {
                transform.P = (x, y);
                Polygon polygon = Geometry.MakeOffsetPolygon(hull, 0.0f, transform);
                Shape.CreatePolygonShape(m_groundId, shapeDef, polygon);
                x += 2.0f * m * hx;
                y += 2.0f * m * hx;
            }
            {
                transform.P = (x, y);
                Polygon polygon = Geometry.MakeOffsetPolygon(hull, 0.0f, transform);
                Shape.CreatePolygonShape(m_groundId, shapeDef, polygon);
                x += 2.0f * m * hx;
                y += 2.0f * m * hx;
            }
            {
                transform.P = (x, y);
                Polygon polygon = Geometry.MakeOffsetPolygon(hull, 0.0f, transform);
                Shape.CreatePolygonShape(m_groundId, shapeDef, polygon);
                x += 2.0f * m * hx;
                y += 2.0f * m * hx;
            }
        }
    }

    protected void Launch()
    {
        if (m_bodyId.IsNotNull)
        {
            Body.DestroyBody(m_bodyId);
            m_shapeId = ShapeId.NullId;
        }

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.Position = (-28.0f, 18.0f);
        bodyDef.LinearVelocity = (0.0f, 0.0f);
        m_bodyId = Body.CreateBody(WorldId, bodyDef);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;
        shapeDef.Friction = m_friction;

        if (m_shapeType == TestShapeType.e_circleShape)
        {
            Circle circle = ((0.0f, 0.0f), 0.5f);
            m_shapeId = Shape.CreateCircleShape(m_bodyId, shapeDef, circle);
        }
        else if (m_shapeType == TestShapeType.e_capsuleShape)
        {
            Capsule capsule = ((-0.5f, 0.0f), (0.5f, 0.0f), 0.25f);
            m_shapeId = Shape.CreateCapsuleShape(m_bodyId, shapeDef, capsule);
        }
        else
        {
            float h = 0.5f - m_round;
            Polygon box = Geometry.MakeRoundedBox(h, 2.0f * h, m_round);
            m_shapeId = Shape.CreatePolygonShape(m_bodyId, shapeDef, box);
        }
    }
}