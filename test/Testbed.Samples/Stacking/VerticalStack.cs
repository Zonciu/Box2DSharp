using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Stacking;

[Sample("Stacking", "Vertical Stack")]
public class VerticalStack : SampleBase
{
    public const int e_maxColumns = 10;

    public const int e_maxRows = 12;

    public const int e_maxBullets = 8;

    public const int e_circleShape = 0;

    public const int e_boxShape = 1;

    public VerticalStack(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (-7.0f, 9.0f);
            Global.Camera.Zoom = 14.0f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Position = (0.0f, -1.0f);
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            Polygon box = Geometry.MakeBox(100.0f, 1.0f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(groundId, shapeDef, box);

            Segment segment = ((10.0f, 1.0f), (10.0f, 21.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        for (int i = 0; i < e_maxRows * e_maxColumns; ++i)
        {
            Bodies[i] = BodyId.NullId;
        }

        for (int i = 0; i < e_maxBullets; ++i)
        {
            Bullets[i] = BodyId.NullId;
        }

        ShapeType = e_boxShape;
        RowCount = e_maxRows;
        ColumnCount = 5;
        BulletCount = 1;
        BulletType = e_circleShape;

        CreateStacks();
    }

    protected void CreateStacks()
    {
        for (int i = 0; i < e_maxRows * e_maxColumns; ++i)
        {
            if (Bodies[i].IsNotNull)
            {
                Body.DestroyBody(Bodies[i]);
                Bodies[i] = BodyId.NullId;
            }
        }

        Circle circle = new();
        circle.Radius = 0.5f;

        Polygon box = Geometry.MakeBox(0.5f, 0.5f);

        // Polygon box = MakeRoundedBox(0.45f, 0.45f, 0.05f);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 1.0f;
        shapeDef.Friction = 0.3f;

        float offset;

        if (ShapeType == e_circleShape)
        {
            offset = 0.0f;
        }
        else
        {
            offset = 0.01f;
        }

        float dx = -3.0f;
        float xroot = 8.0f;

        for (int j = 0; j < ColumnCount; ++j)
        {
            float x = xroot + j * dx;

            for (int i = 0; i < RowCount; ++i)
            {
                BodyDef bodyDef = BodyDef.DefaultBodyDef();
                bodyDef.Type = BodyType.DynamicBody;

                int n = j * RowCount + i;

                float shift = (i % 2 == 0 ? -offset : offset);
                bodyDef.Position = (x + shift, 0.5f + 1.0f * i);

                // bodyDef.Position = (x + shift, 1.0f + 1.51f * i);
                BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

                Bodies[n] = bodyId;

                if (ShapeType == e_circleShape)
                {
                    Shape.CreateCircleShape(bodyId, shapeDef, circle);
                }
                else
                {
                    Shape.CreatePolygonShape(bodyId, shapeDef, box);
                }
            }
        }
    }

    protected void DestroyBody()
    {
        for (int j = 0; j < ColumnCount; ++j)
        {
            for (int i = 0; i < RowCount; ++i)
            {
                int n = j * RowCount + i;

                if ((Bodies[n]).IsNotNull)
                {
                    Body.DestroyBody(Bodies[n]);
                    Bodies[n] = BodyId.NullId;
                    break;
                }
            }
        }
    }

    protected void DestroyBullets()
    {
        for (int i = 0; i < e_maxBullets; ++i)
        {
            BodyId bullet = Bullets[i];

            if (bullet.IsNotNull)
            {
                Body.DestroyBody(bullet);
                Bullets[i] = BodyId.NullId;
            }
        }
    }

    protected void FireBullets()
    {
        Circle circle = ((0.0f, 0.0f), 0.25f);
        Polygon box = Geometry.MakeBox(0.25f, 0.25f);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Density = 4.0f;

        for (int i = 0; i < BulletCount; ++i)
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (-25.0f - i, 6.0f);
            float speed = B2Random.Shared.RandomFloat(200.0f, 300.0f);
            bodyDef.LinearVelocity = (speed, 0.0f);
            bodyDef.IsBullet = true;

            BodyId bullet = Body.CreateBody(WorldId, bodyDef);

            if (BulletType == e_boxShape)
            {
                Shape.CreatePolygonShape(bullet, shapeDef, box);
            }
            else
            {
                Shape.CreateCircleShape(bullet, shapeDef, circle);
            }

            Debug.Assert(Bullets[i].IsNull);
            Bullets[i] = bullet;
        }
    }

    protected BodyId[] Bullets = new BodyId[e_maxBullets];

    protected BodyId[] Bodies = new BodyId[e_maxRows * e_maxColumns];

    protected int ColumnCount;

    protected int RowCount;

    protected int BulletCount;

    protected int ShapeType;

    protected int BulletType;
}