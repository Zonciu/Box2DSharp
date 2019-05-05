using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;

namespace Box2DSharp.Tests
{
    public class PositionTest : Test
    {
        /// <inheritdoc />
        public PositionTest()
        {
            var gshape = new EdgeShape();
            gshape.Set(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));

            var ground = World.CreateBody(new BodyDef() {BodyType = BodyType.StaticBody, Position = new Vector2(0, -5)})
                              .CreateFixture(gshape, 1.0f);
            for (var i = 0; i < 100; i++)
            {
                var b1 = World.CreateBody(
                    new BodyDef() {BodyType = BodyType.DynamicBody, Position = new Vector2(RandomFloat(0, 5), RandomFloat(0, 5))});
                var shape = new CircleShape() {Radius = 1};

                b1.CreateFixture(shape, 1.0f);
            }
        }
    }
}