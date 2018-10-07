using System;
using System.Collections.Generic;
using System.Linq;
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
            var world = new World();

            var groundBodyDef = new BodyDef {BodyType = BodyType.StaticBody};
            groundBodyDef.Position.Set(0.0f, -10.0f);
            var groundBody = world.CreateBody(groundBodyDef);
            var groundBox  = new PolygonShape();
            groundBox.SetAsBox(1000.0f, 10.0f);
            groundBody.CreateFixture(groundBox, 0.0f);

            var bodyDef = new BodyDef {BodyType = BodyType.DynamicBody};
            bodyDef.Position.Set(0, 4f);
            var dynamicBox = new PolygonShape();
            dynamicBox.SetAsBox(1f, 1f, Vector2.Zero, 45f);

            var fixtureDef = new FixtureDef
            {
                shape    = dynamicBox,
                density  = 1.0f,
                friction = 0.3f
            };

            var body = world.CreateBody(bodyDef);
            body.CreateFixture(fixtureDef);

            const float timeStep           = 1.0f / 60.0f;
            const int   velocityIterations = 12;
            const int   positionIterations = 6;

            for (var i = 0; i < int.MaxValue; ++i)
            {
                world.Step(timeStep, velocityIterations, positionIterations);

                var p = body.GetPosition();

                var angle   = body.GetAngle();
                var isAwake = body.IsAwake;
                var v       = body.LinearVelocity;
                var a       = body.AngularVelocity;
                Console.WriteLine($"{p},{angle},{isAwake}, {v}, {a}");
            }
        }
    }
}