using System;
using System.Diagnostics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Dynamics;
using Box2DSharp.Inspection;
using Vector2 = System.Numerics.Vector2;

namespace Box2DSharp.Tests
{
    public class Tiles : TestBase
    {
        [ShowOnly]
        public long CreateTime;

        [ShowOnly]
        public long FixtureCount;

        [ShowOnly]
        public int DynamicTreeHeight;

        [ShowOnly]
        public int MinHeight;

        private const int Count = 20;

        private void Start()
        {
            var fixtureCount = 0;
            var timer = Stopwatch.StartNew();
            {
                var a = 0.5f;
                var bd = new BodyDef();
                bd.Position.Y = -a;
                var ground = World.CreateBody(bd);

                var N = 200;
                var M = 10;
                var position = new Vector2();
                position.Y = 0.0f;
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
                        var body = World.CreateBody(bd);
                        body.CreateFixture(shape, 5.0f);
                        ++fixtureCount;
                        y += deltaY;
                    }

                    x += deltaX;
                }
            }

            CreateTime = timer.ElapsedMilliseconds;
            FixtureCount = fixtureCount;
        }

        protected override void PreStep()
        {
            var cm = World.ContactManager;
            var height = cm.BroadPhase.GetTreeHeight();
            var leafCount = cm.BroadPhase.GetProxyCount();
            var minimumNodeCount = 2 * leafCount - 1;
            var minimumHeight = (int) Math.Ceiling(Math.Log(minimumNodeCount) / Math.Log(2.0f));
            DynamicTreeHeight = height;
            MinHeight = minimumHeight;
        }
    }
}