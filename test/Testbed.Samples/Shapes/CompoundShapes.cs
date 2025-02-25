using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Shapes;

[Sample("Shapes", "Compound Shapes")]
public class CompoundShapes : SampleBase
{
    public CompoundShapes(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 6.0f);
            Global.Camera.Zoom = 25.0f * 0.5f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((50.0f, 0.0f), (-50.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        // Table 1
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (-15.0f, 1.0f);
            Table1Id = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon top = Geometry.MakeOffsetBox(3.0f, 0.5f, (0.0f, 3.5f), Rot.Identity);
            Polygon leftLeg = Geometry.MakeOffsetBox(0.5f, 1.5f, (-2.5f, 1.5f), Rot.Identity);
            Polygon rightLeg = Geometry.MakeOffsetBox(0.5f, 1.5f, (2.5f, 1.5f), Rot.Identity);

            Shape.CreatePolygonShape(Table1Id, shapeDef, top);
            Shape.CreatePolygonShape(Table1Id, shapeDef, leftLeg);
            Shape.CreatePolygonShape(Table1Id, shapeDef, rightLeg);
        }

        // Table 2
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (-5.0f, 1.0f);
            Table2Id = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon top = Geometry.MakeOffsetBox(3.0f, 0.5f, (0.0f, 3.5f), Rot.Identity);
            Polygon leftLeg = Geometry.MakeOffsetBox(0.5f, 2.0f, (-2.5f, 2.0f), Rot.Identity);
            Polygon rightLeg = Geometry.MakeOffsetBox(0.5f, 2.0f, (2.5f, 2.0f), Rot.Identity);

            Shape.CreatePolygonShape(Table2Id, shapeDef, top);
            Shape.CreatePolygonShape(Table2Id, shapeDef, leftLeg);
            Shape.CreatePolygonShape(Table2Id, shapeDef, rightLeg);
        }

        // Spaceship 1
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (5.0f, 1.0f);
            Ship1Id = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Vec2[] vertices = new Vec2[3];

            vertices[0] = (-2.0f, 0.0f);
            vertices[1] = (0.0f, 4.0f / 3.0f);
            vertices[2] = (0.0f, 4.0f);
            Hull hull = HullFunc.ComputeHull(vertices, 3);
            Polygon left = Geometry.MakePolygon(hull, 0.0f);

            vertices[0] = (2.0f, 0.0f);
            vertices[1] = (0.0f, 4.0f / 3.0f);
            vertices[2] = (0.0f, 4.0f);
            hull = HullFunc.ComputeHull(vertices, 3);
            Polygon right = Geometry.MakePolygon(hull, 0.0f);

            Shape.CreatePolygonShape(Ship1Id, shapeDef, left);
            Shape.CreatePolygonShape(Ship1Id, shapeDef, right);
        }

        // Spaceship 2
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = (15.0f, 1.0f);
            Ship2Id = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Vec2[] vertices = new Vec2[3];

            vertices[0] = (-2.0f, 0.0f);
            vertices[1] = (1.0f, 2.0f);
            vertices[2] = (0.0f, 4.0f);
            Hull hull = HullFunc.ComputeHull(vertices, 3);
            Polygon left = Geometry.MakePolygon(hull, 0.0f);

            vertices[0] = (2.0f, 0.0f);
            vertices[1] = (-1.0f, 2.0f);
            vertices[2] = (0.0f, 4.0f);
            hull = HullFunc.ComputeHull(vertices, 3);
            Polygon right = Geometry.MakePolygon(hull, 0.0f);

            Shape.CreatePolygonShape(Ship2Id, shapeDef, left);
            Shape.CreatePolygonShape(Ship2Id, shapeDef, right);
        }

        DrawBodyAABBs = false;
    }

    protected void Spawn()
    {
        // Table 1 obstruction
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = Body.GetPosition(Table1Id);
            bodyDef.Rotation = Body.GetRotation(Table1Id);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeOffsetBox(4.0f, 0.1f, (0.0f, 3.0f), Rot.Identity);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }

        // Table 2 obstruction
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = Body.GetPosition(Table2Id);
            bodyDef.Rotation = Body.GetRotation(Table2Id);
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeOffsetBox(4.0f, 0.1f, (0.0f, 3.0f), Rot.Identity);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);
        }

        // Ship 1 obstruction
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = Body.GetPosition(Ship1Id);
            bodyDef.Rotation = Body.GetRotation(Ship1Id);

            // bodyDef.GravityScale = 0.0f;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Circle circle = ((0.0f, 2.0f), 0.5f);
            Shape.CreateCircleShape(bodyId, shapeDef, circle);
        }

        // Ship 2 obstruction
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.Position = Body.GetPosition(Ship2Id);
            bodyDef.Rotation = Body.GetRotation(Ship2Id);

            // bodyDef.GravityScale = 0.0f;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Circle circle = ((0.0f, 2.0f), 0.5f);
            Shape.CreateCircleShape(bodyId, shapeDef, circle);
        }
    }

    protected override void OnRender()
    {
        if (DrawBodyAABBs)
        {
            AABB aabb = Body.ComputeAABB(Table1Id);
            Draw.DrawAABB(aabb, B2HexColor.Yellow);

            aabb = Body.ComputeAABB(Table2Id);
            Draw.DrawAABB(aabb, B2HexColor.Yellow);

            aabb = Body.ComputeAABB(Ship1Id);
            Draw.DrawAABB(aabb, B2HexColor.Yellow);

            aabb = Body.ComputeAABB(Ship2Id);
            Draw.DrawAABB(aabb, B2HexColor.Yellow);
        }
    }

    protected BodyId Table1Id;

    protected BodyId Table2Id;

    protected BodyId Ship1Id;

    protected BodyId Ship2Id;

    protected bool DrawBodyAABBs;
}