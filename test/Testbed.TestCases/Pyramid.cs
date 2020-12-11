using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Stacking", "Pyramid")]
    public class Pyramid : TestBase
    {
        private const int Count = 20;

        public Pyramid()
        {
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            {
                var a = 0.5f;
                var shape = new PolygonShape();
                shape.SetAsBox(a, a);

                var x = new Vector2(-7.0f, 0.75f);
                Vector2 y;
                var deltaX = new Vector2(0.5625f, 1.25f);
                var deltaY = new Vector2(1.125f, 0.0f);

                for (var i = 0; i < Count; ++i)
                {
                    y = x;

                    for (var j = i; j < Count; ++j)
                    {
                        var bd = new BodyDef();
                        bd.BodyType = BodyType.DynamicBody;
                        bd.Position = y;
                        var body = World.CreateBody(bd);
                        body.CreateFixture(shape, 5.0f);

                        y += deltaY;
                    }

                    x += deltaX;
                }
            }
        }
    }
}