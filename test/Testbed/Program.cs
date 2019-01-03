using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks.Sources;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;

namespace Testbed
{
    internal class Program
    {
        private static void Main()
        {
            Tiles();
        }

        private static void Tiles()
        {
            var world = new World(new Vector2(0, -10));
            long createTime;

            int fixtureCount = 0;

            int dynamicTreeHeight;

            int minHeight;

            const int Count = 20;
            var timer = Stopwatch.StartNew();
            {
                var a = 0.5f;
                var bd = new BodyDef();
                bd.Position.Y = -a;
                var ground = world.CreateBody(bd);

                var N = 200;
                var M = 10;
                var position = new Vector2();
                for (var j = 0; j < M; ++j)
                {
                    position.X = -N * a;
                    for (var i = 0; i < N; ++i)
                    {
                        var shape = new PolygonShape();
                        shape.SetAsBox(a, a, position, 0.0f);
                        ground.CreateFixture(shape, 0.0f);
                        ++fixtureCount;
                        position.X += 2.0f * a;
                    }

                    position.Y -= 2.0f * a;
                }
            }

            {
                var a = 0.5f;
                var shape = new PolygonShape();
                shape.SetAsBox(a, a);

                var x = new Vector2(-7.0f, 0.75f);
                var y = new Vector2();
                var deltaX = new Vector2(0.5625f, 1.25f);
                var deltaY = new Vector2(1.125f, 0.0f);

                for (int i = 0; i < Count; ++i)
                {
                    y = x;

                    for (var j = i; j < Count; ++j)
                    {
                        var bd = new BodyDef {BodyType = BodyType.DynamicBody, Position = y};
                        var body = world.CreateBody(bd);
                        body.CreateFixture(shape, 5.0f);
                        ++fixtureCount;
                        y += deltaY;
                    }

                    x += deltaX;
                }
            }
            timer.Stop();
            createTime = timer.ElapsedMilliseconds;

            {
                var cm = world.ContactManager;
                var height = cm.BroadPhase.GetTreeHeight();
                var leafCount = cm.BroadPhase.GetProxyCount();
                var minimumNodeCount = 2 * leafCount - 1;
                var minimumHeight = (int) Math.Ceiling(Math.Log(minimumNodeCount) / Math.Log(2.0f));
                dynamicTreeHeight = height;
                minHeight = minimumHeight;
            }
            
            var dt = 1 / 60;

            timer = Stopwatch.StartNew();
            for (var i = 0; i < 36000; ++i)
            {
                world.Step(dt, 8, 3);
            }

            timer.Stop();

            Console.WriteLine($"Stop {timer.ElapsedMilliseconds}");
            Console.WriteLine($"Create time {createTime}, fixture {fixtureCount}");
            Console.WriteLine($"Height {dynamicTreeHeight}, min height {minHeight}");
        }

        private static void HelloWorld()
        {
            var world = new World();

            var groundBodyDef = new BodyDef {BodyType = BodyType.StaticBody};
            groundBodyDef.Position.Set(0.0f, -10.0f);
            var groundBody = world.CreateBody(groundBodyDef);
            var groundBox = new PolygonShape();
            groundBox.SetAsBox(1000.0f, 10.0f);
            groundBody.CreateFixture(groundBox, 0.0f);

            var bodyDef = new BodyDef {BodyType = BodyType.DynamicBody};
            bodyDef.Position.Set(0, 4f);
            var dynamicBox = new PolygonShape();
            dynamicBox.SetAsBox(1f, 1f, Vector2.Zero, 45f);

            var fixtureDef = new FixtureDef
            {
                Shape = dynamicBox,
                Density = 1.0f,
                Friction = 0.3f
            };

            var body = world.CreateBody(bodyDef);
            body.CreateFixture(fixtureDef);

            const float timeStep = 1.0f / 60.0f;
            const int velocityIterations = 12;
            const int positionIterations = 6;

            for (var i = 0; i < int.MaxValue; ++i)
            {
                world.Step(timeStep, velocityIterations, positionIterations);

                var p = body.GetPosition();

                var angle = body.GetAngle();
                var isAwake = body.IsAwake;
                var v = body.LinearVelocity;
                var a = body.AngularVelocity;
                Console.WriteLine($"{p},{angle},{isAwake}, {v}, {a}");
            }
        }
    }
}