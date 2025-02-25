using System;
using Box2DSharp;
using FluentAssertions;
using Xunit;

namespace UnitTest;

public class WorldTest
{
    // This is a simple example of building and running a simulation
    // using Box2D. Here we create a large ground box and a small dynamic
    // box.
    // There are no graphics for this example. Box2D is meant to be used
    // with your rendering engine in your game engine.
    [Fact]
    public int HelloWorld()
    {
        // Construct a world object, which will hold and simulate the rigid bodies.
        WorldDef worldDef = WorldDef.DefaultWorldDef();
        worldDef.Gravity = (0.0f, -10.0f);

        WorldId worldId = World.CreateWorld(worldDef);
        World.World_IsValid(worldId).Should().BeTrue();

        // Define the ground body.
        BodyDef groundBodyDef = BodyDef.DefaultBodyDef();
        groundBodyDef.Position = (0.0f, -10.0f);

        // Call the body factory which allocates memory for the ground body
        // from a pool and creates the ground box shape (also from a pool).
        // The body is also added to the world.
        BodyId groundId = Body.CreateBody(worldId, groundBodyDef);
        World.Body_IsValid(groundId).Should().BeTrue();

        // Define the ground box shape. The extents are the half-widths of the box.
        Polygon groundBox = Geometry.MakeBox(50.0f, 10.0f);

        // Add the box shape to the ground body.
        ShapeDef groundShapeDef = ShapeDef.DefaultShapeDef();
        Shape.CreatePolygonShape(groundId, groundShapeDef, groundBox);

        // Define the dynamic body. We set its position and call the body factory.
        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.Position = (0.0f, 4.0f);

        BodyId bodyId = Body.CreateBody(worldId, bodyDef);

        // Define another box shape for our dynamic body.
        Polygon dynamicBox = Geometry.MakeBox(1.0f, 1.0f);

        // Define the dynamic body shape
        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

        // Set the box density to be non-zero, so it will be dynamic.
        shapeDef.Density = 1.0f;

        // Override the default friction.
        shapeDef.Friction = 0.3f;

        // Add the shape to the body.
        Shape.CreatePolygonShape(bodyId, shapeDef, dynamicBox);

        // Prepare for simulation. Typically we use a time step of 1/60 of a
        // second (60Hz) and 4 sub-steps. This provides a high quality simulation
        // in most game scenarios.
        float timeStep = 1.0f / 60.0f;
        int subStepCount = 4;

        Vec2 position = Body.GetPosition(bodyId);
        Rot rotation = Body.GetRotation(bodyId);

        // This is our little game loop.
        for (int i = 0; i < 90; ++i)
        {
            // Instruct the world to perform a single step of simulation.
            // It is generally best to keep the time step and iterations fixed.
            World.Step(worldId, timeStep, subStepCount);

            // Now print the position and angle of the body.
            position = Body.GetPosition(bodyId);
            rotation = Body.GetRotation(bodyId);

            // printf("%4.2f %4.2f %4.2f\n", position.x, position.y, Rot_GetAngle(rotation));
        }

        // When the world destructor is called, all bodies and joints are freed. This can
        // create orphaned ids, so be careful about your world management.
        World.DestroyWorld(worldId);
        MathF.Abs(position.X).Should().BeLessThan(0.01f);
        MathF.Abs(position.Y - 1f).Should().BeLessThan(0.01f);
        MathF.Abs(rotation.GetAngle()).Should().BeLessThan(0.01f);

        return 0;
    }

    [Fact]
    public int EmptyWorld()
    {
        WorldDef worldDef = WorldDef.DefaultWorldDef();
        WorldId worldId = World.CreateWorld(worldDef);
        World.World_IsValid(worldId).Should().BeTrue();

        float timeStep = 1.0f / 60.0f;
        var subStepCount = 1;

        for (var i = 0; i < 60; ++i)
        {
            World.Step(worldId, timeStep, subStepCount);
        }

        World.DestroyWorld(worldId);
        World.World_IsValid(worldId).Should().BeFalse();

        return 0;
    }

    public const int BODY_COUNT = 10;

    [Fact]
    public int DestroyAllBodiesWorld()
    {
        WorldDef worldDef = WorldDef.DefaultWorldDef();
        WorldId worldId = World.CreateWorld(worldDef);

        World.World_IsValid(worldId).Should().BeTrue();

        int count = 0;
        bool creating = true;

        BodyId[] bodyIds = new BodyId[BODY_COUNT];
        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        Polygon square = Geometry.MakeSquare(0.5f);

        for (var i = 0; i < 2 * BODY_COUNT + 10; ++i)
        {
            if (creating)
            {
                if (count < BODY_COUNT)
                {
                    bodyIds[count] = Body.CreateBody(worldId, bodyDef);

                    ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
                    Shape.CreatePolygonShape(bodyIds[count], shapeDef, square);
                    count += 1;
                }
                else
                {
                    creating = false;
                }
            }
            else if (count > 0)
            {
                Body.DestroyBody(bodyIds[count - 1]);
                bodyIds[count - 1] = BodyId.NullId;
                count -= 1;
            }

            World.Step(worldId, 1.0f / 60.0f, 3);
        }

        Counters counters = World.GetCounters(worldId);
        counters.BodyCount.Should().Be(0);

        World.DestroyWorld(worldId);

        World.World_IsValid(worldId).Should().BeFalse();

        return 0;
    }

    [Fact]
    public int TestIsValid()
    {
        WorldDef worldDef = WorldDef.DefaultWorldDef();
        WorldId worldId = World.CreateWorld(worldDef);
        World.World_IsValid(worldId).Should().BeTrue();

        BodyDef bodyDef = BodyDef.DefaultBodyDef();

        BodyId bodyId1 = Body.CreateBody(worldId, bodyDef);
        World.Body_IsValid(bodyId1).Should().BeTrue();

        BodyId bodyId2 = Body.CreateBody(worldId, bodyDef);
        World.Body_IsValid(bodyId2).Should().BeTrue();

        Body.DestroyBody(bodyId1);
        World.Body_IsValid(bodyId1).Should().BeFalse();

        Body.DestroyBody(bodyId2);
        World.Body_IsValid(bodyId2).Should().BeFalse();

        World.DestroyWorld(worldId);
        World.World_IsValid(worldId).Should().BeFalse();
        World.Body_IsValid(bodyId1).Should().BeFalse();
        World.Body_IsValid(bodyId2).Should().BeFalse();

        return 0;
    }

    [Fact]
    public int TestForAmy()
    {
        {
            WorldDef worldDef = WorldDef.DefaultWorldDef();
            WorldId world_id = World.CreateWorld(worldDef);

            BodyDef body_def = BodyDef.DefaultBodyDef();

            body_def.Type = BodyType.StaticBody;
            body_def.Position = (0, 0);
            BodyId body_id = Body.CreateBody(world_id, body_def);
            Polygon polygon = Geometry.MakeBox(1, 1);
            ShapeDef shape_def = ShapeDef.DefaultShapeDef();
            Shape.CreatePolygonShape(body_id, shape_def, polygon);

            BodyDef simulon_body_def = BodyDef.DefaultBodyDef();

            simulon_body_def.Position = (0, -7.5f);
            simulon_body_def.Type = BodyType.DynamicBody;

            BodyId simulon_body_id = Body.CreateBody(world_id, simulon_body_def);
            Circle ball = ((0.0f, 0.35f), 0.5f);

            ShapeDef simulon_shape_def = ShapeDef.DefaultShapeDef();
            Shape.CreateCircleShape(simulon_body_id, simulon_shape_def, ball);

            Polygon the_box = Geometry.MakeRoundedBox(0.1f, 0.1f, 0.01f);
            Shape.CreatePolygonShape(simulon_body_id, simulon_shape_def, the_box);
            BodyDef head_body_def = BodyDef.DefaultBodyDef();
            head_body_def.Position = (0, 6);
            head_body_def.Type = BodyType.DynamicBody;
            BodyId head_body_id = Body.CreateBody(world_id, head_body_def);
            RevoluteJointDef joint_def5 = RevoluteJointDef.DefaultRevoluteJointDef();
            joint_def5.BodyIdA = simulon_body_id;
            joint_def5.BodyIdB = head_body_id;
            joint_def5.LocalAnchorA = (0.0f, 0.8f);
            joint_def5.LocalAnchorB = (0.0f, -0.17f / 2.0f);

            JointId revolute_joint_id = Joint.CreateRevoluteJoint(world_id, joint_def5);
            DistanceJointDef joint_def6 = DistanceJointDef.DefaultDistanceJointDef();
            joint_def6.BodyIdA = simulon_body_id;
            joint_def6.BodyIdB = head_body_id;
            joint_def6.LocalAnchorA = (0.0f, 1.7f);
            joint_def6.LocalAnchorB = (0.0f, 0.8f);
            joint_def6.Length = 0.005f;
            joint_def6.Hertz = 1f;
            Joint.CreateDistanceJoint(world_id, joint_def6);

            Body.DestroyBody(simulon_body_id);

            World.Step(world_id, 1f / 60f, 4);

            World.DestroyWorld(world_id);
        }

        {
            WorldDef worldDef = WorldDef.DefaultWorldDef();
            WorldId world_id = World.CreateWorld(worldDef);

            BodyDef ground_body_def = BodyDef.DefaultBodyDef();
            ground_body_def.Type = BodyType.StaticBody;
            BodyId ground_body_id = Body.CreateBody(world_id, ground_body_def);

            BodyDef box_body_def = BodyDef.DefaultBodyDef();
            box_body_def.Type = BodyType.DynamicBody;
            box_body_def.Position = (0.0f, 0.0f);
            BodyId box_body_id = Body.CreateBody(world_id, box_body_def);
            Polygon polygon = Geometry.MakeBox(1f, 1f);
            ShapeDef shape_def = ShapeDef.DefaultShapeDef();
            ShapeId box_shape = Shape.CreatePolygonShape(box_body_id, shape_def, polygon);

            DistanceJointDef distance_joint_def = DistanceJointDef.DefaultDistanceJointDef();
            distance_joint_def.Hertz = 1f;
            distance_joint_def.DampingRatio = 0.1f;
            distance_joint_def.BodyIdA = ground_body_id;
            distance_joint_def.BodyIdB = box_body_id;
            distance_joint_def.MinLength = 0.005f;
            distance_joint_def.EnableSpring = true;
            distance_joint_def.EnableLimit = false;
            distance_joint_def.CollideConnected = false;
            distance_joint_def.Length = 0.005f;
            Body.SetTransform(ground_body_id, (0.0f, 0.0f), (1f, 0f));
            distance_joint_def.LocalAnchorA = Vec2.Zero;
            distance_joint_def.LocalAnchorB = Vec2.Zero;

            JointId distance_joint_id = Joint.CreateDistanceJoint(world_id, distance_joint_def);

            Body.SetType(box_body_id, BodyType.StaticBody);
            World.Step(world_id, 1f / 60f, 4);

            Joint.DestroyJoint(distance_joint_id);

            World.DestroyWorld(world_id);
        }

        return 0;
    }
}