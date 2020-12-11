using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Stacking", "Circles")]
    public class CircleStack : TestBase
    {
        private const int Count = 10;

        private readonly Body[] _bodies = new Body[Count];

        public CircleStack()
        {
            {
                var bd = new BodyDef();
                var ground = World.CreateBody(bd);

                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            {
                var shape = new CircleShape {Radius = 1.0f};

                for (var i = 0; i < Count; ++i)
                {
                    var bd = new BodyDef
                    {
                        BodyType = BodyType.DynamicBody, Position = new Vector2(0.0f, 4.0f + 3.0f * i)
                    };

                    _bodies[i] = World.CreateBody(bd);

                    _bodies[i].CreateFixture(shape, 1.0f);

                    _bodies[i].SetLinearVelocity(new Vector2(0.0f, -50.0f));
                }
            }
        }
    }
}