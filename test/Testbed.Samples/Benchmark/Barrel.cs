using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Benchmark;

[Sample("Benchmark", "Barrel")]
public class Barrel : SampleBase
{
    public enum TestShapeType
    {
        CircleShape = 0,

        CapsuleShape,

        MixShape,

        CompoundShape,

        HumanShape,
    }

    public const int MaxColumns = 26;

    public const int MaxRows = 130;

    protected BodyId[] Bodies = new BodyId [MaxRows * MaxColumns];

    protected int ColumnCount;

    protected int RowCount;

    protected TestShapeType ShapeType;

    protected Human[] Humans = new Human[MaxRows * MaxColumns].Fill();

    public Barrel(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (8.0f, 53.0f);
            Global.Camera.Zoom = 25.0f * 2.35f;
        }

        settings.DrawJoints = false;

        float groundSize = 25.0f;

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(groundSize, 1.2f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(1.2f, 2.0f * groundSize, (-groundSize, 2.0f * groundSize), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(1.2f, 2.0f * groundSize, (groundSize, 2.0f * groundSize), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            box = Geometry.MakeOffsetBox(800.0f, 10.0f, (0.0f, -80.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        for (int i = 0; i < MaxRows * MaxColumns; ++i)
        {
            Bodies[i] = BodyId.NullId;
        }

        ShapeType = TestShapeType.CircleShape;

        CreateScene();
    }

    protected void CreateScene()
    {
        for (int i = 0; i < MaxRows * MaxColumns; ++i)
        {
            if (Bodies[i].IsNotNull)
            {
                Body.DestroyBody(Bodies[i]);
                Bodies[i] = BodyId.NullId;
            }

            if (Humans[i].IsSpawned)
            {
                Humans[i].Despawn();
            }
        }

        ColumnCount = Core.B2Debug ? 10 : MaxColumns;
        RowCount = Core.B2Debug ? 40 : MaxRows;

        if (ShapeType == TestShapeType.CompoundShape)
        {
            if (Core.B2Debug == false)
            {
                ColumnCount = 20;
            }
        }
        else if (ShapeType == TestShapeType.HumanShape)
        {
            if (Core.B2Debug)
            {
                RowCount = 5;
                ColumnCount = 10;
            }
            else
            {
                ColumnCount = 15;
                RowCount = 50;
            }
        }

        float rad = 0.5f;

        float shift = 1.15f;
        float centerx = shift * ColumnCount / 2.0f;
        float centery = shift / 2.0f;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;
        shapeDef.Friction = 0.5f;

        Capsule capsule = ((0.0f, -0.25f), (0.0f, 0.25f), rad);
        Circle circle = ((0.0f, 0.0f), rad);

        Vec2[] points = [(-0.1f, -0.5f), (0.1f, -0.5f), (0.0f, 0.5f)];
        Hull wedgeHull = HullFunc.ComputeHull(points, 3);
        Polygon wedge = Geometry.MakePolygon(wedgeHull, 0.0f);

        Vec2[] vertices = new Vec2[3];
        vertices[0] = (-1.0f, 0.0f);
        vertices[1] = (0.5f, 1.0f);
        vertices[2] = (0.0f, 2.0f);
        Hull hull = HullFunc.ComputeHull(vertices, 3);
        Polygon left = Geometry.MakePolygon(hull, 0.0f);

        vertices[0] = (1.0f, 0.0f);
        vertices[1] = (-0.5f, 1.0f);
        vertices[2] = (0.0f, 2.0f);
        hull = HullFunc.ComputeHull(vertices, 3);
        Polygon right = Geometry.MakePolygon(hull, 0.0f);

        // Polygon top = Geometry.MakeOffsetBox(0.8f, 0.2f, {0.0f, 0.8f}, 0.0f);
        // Polygon leftLeg = Geometry.MakeOffsetBox(0.2f, 0.5f, {-0.6f, 0.5f}, 0.0f);
        // Polygon rightLeg = Geometry.MakeOffsetBox(0.2f, 0.5f, {0.6f, 0.5f}, 0.0f);

        float side = -0.1f;
        float extray = 0.5f;

        if (ShapeType == TestShapeType.CompoundShape)
        {
            extray = 0.25f;
            side = 0.25f;
            shift = 2.0f;
            centerx = shift * ColumnCount / 2.0f - 1.0f;
        }
        else if (ShapeType == TestShapeType.HumanShape)
        {
            extray = 0.5f;
            side = 0.55f;
            shift = 2.5f;
            centerx = shift * ColumnCount / 2.0f;
        }

        int index = 0;

        for (int i = 0; i < ColumnCount; ++i)
        {
            float x = i * shift - centerx;

            for (int j = 0; j < RowCount; ++j)
            {
                float y = j * (shift + extray) + centery + 2.0f;

                bodyDef.Position = (x + side, y);
                side = -side;

                if (ShapeType == TestShapeType.CircleShape)
                {
                    Bodies[index] = Body.CreateBody(WorldId, bodyDef);
                    circle.Radius = RandomFloat(0.25f, 0.75f);
                    Shape.CreateCircleShape(Bodies[index], shapeDef, circle);
                }
                else if (ShapeType == TestShapeType.CapsuleShape)
                {
                    Bodies[index] = Body.CreateBody(WorldId, bodyDef);
                    capsule.Radius = RandomFloat(0.25f, 0.5f);
                    float length = RandomFloat(0.25f, 1.0f);
                    capsule.Center1 = (0.0f, -0.5f * length);
                    capsule.Center2 = (0.0f, 0.5f * length);
                    Shape.CreateCapsuleShape(Bodies[index], shapeDef, capsule);
                }
                else if (ShapeType == TestShapeType.MixShape)
                {
                    Bodies[index] = Body.CreateBody(WorldId, bodyDef);

                    int mod = index % 3;
                    if (mod == 0)
                    {
                        circle.Radius = RandomFloat(0.25f, 0.75f);
                        Shape.CreateCircleShape(Bodies[index], shapeDef, circle);
                    }
                    else if (mod == 1)
                    {
                        capsule.Radius = RandomFloat(0.25f, 0.5f);
                        float length = RandomFloat(0.25f, 1.0f);
                        capsule.Center1 = (0.0f, -0.5f * length);
                        capsule.Center2 = (0.0f, 0.5f * length);
                        Shape.CreateCapsuleShape(Bodies[index], shapeDef, capsule);
                    }
                    else if (mod == 2)
                    {
                        float width = RandomFloat(0.1f, 0.5f);
                        float height = RandomFloat(0.5f, 0.75f);
                        Polygon box = Geometry.MakeBox(width, height);

                        // Don't put a function call into a macro.
                        float value = RandomFloat(-1.0f, 1.0f);
                        box.Radius = 0.25f * MathF.Max(0.0f, value);
                        Shape.CreatePolygonShape(Bodies[index], shapeDef, box);
                    }
                    else
                    {
                        wedge.Radius = RandomFloat(0.1f, 0.25f);
                        Shape.CreatePolygonShape(Bodies[index], shapeDef, wedge);
                    }
                }
                else if (ShapeType == TestShapeType.CompoundShape)
                {
                    Bodies[index] = Body.CreateBody(WorldId, bodyDef);

                    Shape.CreatePolygonShape(Bodies[index], shapeDef, left);
                    Shape.CreatePolygonShape(Bodies[index], shapeDef, right);

                    // Shape.CreatePolygonShape(m_bodies[index], shapeDef, &top);
                    // Shape.CreatePolygonShape(m_bodies[index], shapeDef, &leftLeg);
                    // Shape.CreatePolygonShape(m_bodies[index], shapeDef, &rightLeg);
                }
                else if (ShapeType == TestShapeType.HumanShape)
                {
                    Humans[index].Spawn(WorldId, bodyDef.Position, 3.5f, 0.05f, 0.0f, 0.0f, index + 1, null!, false);
                }

                index += 1;
            }
        }
    }
}