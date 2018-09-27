using System;
using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;

namespace Testbed
{
    internal class Program
    {
        private static void Main()
        {
            float a1 = 0.5f, a2 = 1.6f;
            MathUtils.Swap(ref a1, ref a2);

            // Define the gravity vector.

            // Construct a world object, which will hold and simulate the rigid bodies.
            var world = new World();

            // Define the ground body.
            var groundBodyDef = new BodyDef();
            groundBodyDef.Position.Set(0.0f, -10.0f);

            // Call the body factory which allocates memory for the ground body
            // from a pool and creates the ground box shape (also from a pool).
            // The body is also added to the world.
            var groundBody = world.CreateBody(groundBodyDef);

            // Define the ground box shape.
            var groundBox = new PolygonShape();

            // The extents are the half-widths of the box.
            groundBox.SetAsBox(1000.0f, 10.0f);

            // Add the ground fixture to the ground body.
            groundBody.CreateFixture(groundBox, 0.0f);

            // Define the dynamic body. We set its position and call the body factory.
            var bodyDef = new BodyDef {BodyType = BodyType.DynamicBody};

            bodyDef.Position.Set(0, 4f);
            var body = world.CreateBody(bodyDef);

            // Define another box shape for our dynamic body.
            var dynamicBox = new PolygonShape();
            dynamicBox.SetAsBox(1.0f, 1.0f);

            // Define the dynamic body fixture.
            var fixtureDef = new FixtureDef
            {
                shape    = dynamicBox,
                density  = 1.0f,
                friction = 0.3f
            };

            // Set the box density to be non-zero, so it will be dynamic.

            // Override the default friction.

            // Add the shape to the body.
            body.CreateFixture(fixtureDef);

            // Prepare for simulation. Typically we use a time step of 1/60 of a
            // second (60Hz) and 10 iterations. This provides a high quality simulation
            // in most game scenarios.
            var timeStep           = 1.0f / 60.0f;
            var velocityIterations = 6;
            var positionIterations = 2;

            // This is our little game loop.
            for (var i = 0; i < 100; ++i)
            {
                // Console.WriteLine($"Step {i}");
                // body.Dump();

                // Instruct the world to perform a single step of simulation.
                // It is generally best to keep the time step and iterations fixed.
                world.Step(timeStep, velocityIterations, positionIterations);

                // Now print the position and angle of the body.
                var position = body.GetPosition();
                var angle    = body.GetAngle();

                Console.WriteLine($"{position}, {angle}");
            }
        }
    }
}