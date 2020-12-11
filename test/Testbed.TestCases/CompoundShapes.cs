using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Examples", "Compound Shapes")]
    public class CompoundShapes : TestBase
    {
        Body m_table1;

        Body m_table2;

        Body m_ship1;

        Body m_ship2;

        public CompoundShapes()
        {
            {
                BodyDef bd = new BodyDef();
                bd.Position.Set(0.0f, 0.0f);
                Body body = World.CreateBody(bd);

                EdgeShape shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(50.0f, 0.0f), new Vector2(-50.0f, 0.0f));

                body.CreateFixture(shape, 0.0f);
            }

            // Table 1
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(-15.0f, 1.0f);
                m_table1 = World.CreateBody(bd);

                PolygonShape top = new PolygonShape();
                top.SetAsBox(3.0f, 0.5f, new Vector2(0.0f, 3.5f), 0.0f);

                PolygonShape leftLeg = new PolygonShape();
                leftLeg.SetAsBox(0.5f, 1.5f, new Vector2(-2.5f, 1.5f), 0.0f);

                PolygonShape rightLeg = new PolygonShape();
                rightLeg.SetAsBox(0.5f, 1.5f, new Vector2(2.5f, 1.5f), 0.0f);

                m_table1.CreateFixture(top, 2.0f);
                m_table1.CreateFixture(leftLeg, 2.0f);
                m_table1.CreateFixture(rightLeg, 2.0f);
            }

            // Table 2
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(-5.0f, 1.0f);
                m_table2 = World.CreateBody(bd);

                PolygonShape top = new PolygonShape();
                top.SetAsBox(3.0f, 0.5f, new Vector2(0.0f, 3.5f), 0.0f);

                PolygonShape leftLeg = new PolygonShape();
                leftLeg.SetAsBox(0.5f, 2.0f, new Vector2(-2.5f, 2.0f), 0.0f);

                PolygonShape rightLeg = new PolygonShape();
                rightLeg.SetAsBox(0.5f, 2.0f, new Vector2(2.5f, 2.0f), 0.0f);

                m_table2.CreateFixture(top, 2.0f);
                m_table2.CreateFixture(leftLeg, 2.0f);
                m_table2.CreateFixture(rightLeg, 2.0f);
            }

            // Spaceship 1
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(5.0f, 1.0f);
                m_ship1 = World.CreateBody(bd);

                Vector2[] vertices = new Vector2[3];

                PolygonShape left = new PolygonShape();
                vertices[0].Set(-2.0f, 0.0f);
                vertices[1].Set(0.0f, 4.0f / 3.0f);
                vertices[2].Set(0.0f, 4.0f);
                left.Set(vertices, 3);

                PolygonShape right = new PolygonShape();
                vertices[0].Set(2.0f, 0.0f);
                vertices[1].Set(0.0f, 4.0f / 3.0f);
                vertices[2].Set(0.0f, 4.0f);
                right.Set(vertices, 3);

                m_ship1.CreateFixture(left, 2.0f);
                m_ship1.CreateFixture(right, 2.0f);
            }

            // Spaceship 2
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(15.0f, 1.0f);
                m_ship2 = World.CreateBody(bd);

                Vector2[] vertices = new Vector2[3];

                PolygonShape left = new PolygonShape();
                vertices[0].Set(-2.0f, 0.0f);
                vertices[1].Set(1.0f, 2.0f);
                vertices[2].Set(0.0f, 4.0f);
                left.Set(vertices, 3);

                PolygonShape right = new PolygonShape();
                vertices[0].Set(2.0f, 0.0f);
                vertices[1].Set(-1.0f, 2.0f);
                vertices[2].Set(0.0f, 4.0f);
                right.Set(vertices, 3);

                m_ship2.CreateFixture(left, 2.0f);
                m_ship2.CreateFixture(right, 2.0f);
            }
        }

        protected void Spawn()
        {
            // Table 1 obstruction
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position = m_table1.GetPosition();
                bd.Angle = m_table1.GetAngle();

                Body body = World.CreateBody(bd);

                PolygonShape box = new PolygonShape();
                box.SetAsBox(4.0f, 0.1f, new Vector2(0.0f, 3.0f), 0.0f);

                body.CreateFixture(box, 2.0f);
            }

            // Table 2 obstruction
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position = m_table2.GetPosition();
                bd.Angle = m_table2.GetAngle();

                Body body = World.CreateBody(bd);

                PolygonShape box = new PolygonShape();
                box.SetAsBox(4.0f, 0.1f, new Vector2(0.0f, 3.0f), 0.0f);

                body.CreateFixture(box, 2.0f);
            }

            // Ship 1 obstruction
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position = m_ship1.GetPosition();
                bd.Angle = m_ship1.GetAngle();
                bd.GravityScale = 0.0f;

                Body body = World.CreateBody(bd);

                CircleShape circle = new CircleShape();
                circle.Radius = 0.5f;
                circle.Position.Set(0.0f, 2.0f);

                body.CreateFixture(circle, 2.0f);
            }

            // Ship 2 obstruction
            {
                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position = m_ship2.GetPosition();
                bd.Angle = m_ship2.GetAngle();
                bd.GravityScale = 0.0f;

                Body body = World.CreateBody(bd);

                CircleShape circle = new CircleShape();
                circle.Radius = 0.5f;
                circle.Position.Set(0.0f, 2.0f);

                body.CreateFixture(circle, 2.0f);
            }
        }
    }
}