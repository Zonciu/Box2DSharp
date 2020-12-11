using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Bugs", "Chain Problem")]
    public class ChainProblem : TestBase
    {
        public ChainProblem()
        {
            Vector2 g = new Vector2(0.0f, -10.0f);
            World.Gravity = g;
            var bodies = new Body[2];
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.StaticBody;
                bodies[0] = World.CreateBody(bd);

                {
                    var v1 = new Vector2(0.0f, 1.0f);
                    var v2 = new Vector2(0.0f, 0.0f);
                    var v3 = new Vector2(4.0f, 0.0f);

                    EdgeShape shape = new EdgeShape();
                    shape.SetTwoSided(v1, v2);
                    bodies[0].CreateFixture(shape, 0.0f);

                    shape.SetTwoSided(v2, v3);
                    bodies[0].CreateFixture(shape, 0.0f);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;

                //bd.position.Set(6.033980250358582e-01f, 3.028350114822388e+00f);
                bd.Position.Set(1.0f, 3.0f);
                bodies[1] = World.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.Friction = 0.2f;
                    fd.Density = 10.0f;
                    PolygonShape shape = new PolygonShape();
                    var vs = new Vector2[8];
                    vs[0].Set(0.5f, -3.0f);
                    vs[1].Set(0.5f, 3.0f);
                    vs[2].Set(-0.5f, 3.0f);
                    vs[3].Set(-0.5f, -3.0f);
                    shape.Set(vs, 4);

                    fd.Shape = shape;

                    bodies[1].CreateFixture(fd);
                }
            }
            bodies = default;
        }
    }
}