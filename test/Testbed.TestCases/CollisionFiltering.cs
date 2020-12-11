using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Examples", "Collision Filtering")]
    public class CollisionFiltering : TestBase
    {
        private const short SmallGroup = 1;

        private const short LargeGroup = -1;

        private const ushort DefaultCategory = 0x0001;

        private const ushort TriangleCategory = 0x0002;

        private const ushort BoxCategory = 0x0004;

        private const ushort CircleCategory = 0x0008;

        private const ushort TriangleMask = 0xFFFF;

        private const ushort BoxMask = 0xFFFF ^ TriangleCategory;

        private const ushort CircleMask = 0xFFFF;

        public CollisionFiltering()
        {
            {
                // Ground body
                {
                    var shape = new EdgeShape();
                    shape.SetTwoSided(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));

                    var sd = new FixtureDef();
                    sd.Shape = shape;
                    sd.Friction = 0.3f;

                    var bd = new BodyDef();
                    var ground = World.CreateBody(bd);
                    ground.CreateFixture(sd);
                }

                // Small triangle
                var vertices = new Vector2[3];
                vertices[0].Set(-1.0f, 0.0f);
                vertices[1].Set(1.0f, 0.0f);
                vertices[2].Set(0.0f, 2.0f);
                var polygon = new PolygonShape();
                polygon.Set(vertices);

                var triangleShapeDef = new FixtureDef();
                triangleShapeDef.Shape = polygon;
                triangleShapeDef.Density = 1.0f;
                triangleShapeDef.Filter.GroupIndex = SmallGroup;
                triangleShapeDef.Filter.CategoryBits = TriangleCategory;
                triangleShapeDef.Filter.MaskBits = TriangleMask;

                var triangleBodyDef = new BodyDef();
                triangleBodyDef.BodyType = BodyType.DynamicBody;
                triangleBodyDef.Position.Set(-5.0f, 2.0f);

                var body1 = World.CreateBody(triangleBodyDef);
                body1.CreateFixture(triangleShapeDef);

                // Large triangle (recycle definitions)
                vertices[0] *= 2.0f;
                vertices[1] *= 2.0f;
                vertices[2] *= 2.0f;
                polygon.Set(vertices);
                triangleShapeDef.Filter.GroupIndex = LargeGroup;
                triangleBodyDef.Position.Set(-5.0f, 6.0f);
                triangleBodyDef.FixedRotation = true; // look at me!

                var body2 = World.CreateBody(triangleBodyDef);
                body2.CreateFixture(triangleShapeDef);

                {
                    var bd = new BodyDef();
                    bd.BodyType = BodyType.DynamicBody;
                    bd.Position.Set(-5.0f, 10.0f);
                    var body = World.CreateBody(bd);

                    var p = new PolygonShape();
                    p.SetAsBox(0.5f, 1.0f);
                    body.CreateFixture(p, 1.0f);

                    var jd = new PrismaticJointDef();
                    jd.BodyA = body2;
                    jd.BodyB = body;
                    jd.EnableLimit = true;
                    jd.LocalAnchorA.Set(0.0f, 4.0f);
                    jd.LocalAnchorB.SetZero();
                    jd.LocalAxisA.Set(0.0f, 1.0f);
                    jd.LowerTranslation = -1.0f;
                    jd.UpperTranslation = 1.0f;

                    World.CreateJoint(jd);
                }

                // Small box
                polygon.SetAsBox(1.0f, 0.5f);
                var boxShapeDef = new FixtureDef();
                boxShapeDef.Shape = polygon;
                boxShapeDef.Density = 1.0f;
                boxShapeDef.Restitution = 0.1f;
                boxShapeDef.Filter.GroupIndex = SmallGroup;
                boxShapeDef.Filter.CategoryBits = BoxCategory;
                boxShapeDef.Filter.MaskBits = BoxMask;

                var boxBodyDef = triangleBodyDef;
                boxBodyDef.BodyType = BodyType.DynamicBody;
                boxBodyDef.Position.Set(0.0f, 2.0f);

                var body3 = World.CreateBody(boxBodyDef);
                body3.CreateFixture(boxShapeDef);

                // Large box (recycle definitions)
                polygon.SetAsBox(2.0f, 1.0f);
                boxShapeDef.Filter.GroupIndex = LargeGroup;
                boxBodyDef.Position.Set(0.0f, 6.0f);

                var body4 = World.CreateBody(boxBodyDef);
                body4.CreateFixture(boxShapeDef);

                // Small circle
                var circle = new CircleShape();
                circle.Radius = 1.0f;

                var circleShapeDef = new FixtureDef();
                circleShapeDef.Shape = circle;
                circleShapeDef.Density = 1.0f;
                circleShapeDef.Filter.GroupIndex = SmallGroup;
                circleShapeDef.Filter.CategoryBits = CircleCategory;
                circleShapeDef.Filter.MaskBits = CircleMask;

                var circleBodyDef = new BodyDef();
                circleBodyDef.BodyType = BodyType.DynamicBody;
                circleBodyDef.Position.Set(5.0f, 2.0f);

                var body5 = World.CreateBody(circleBodyDef);
                body5.CreateFixture(circleShapeDef);

                // Large circle
                circle.Radius *= 2.0f;
                circleShapeDef.Filter.GroupIndex = LargeGroup;
                circleBodyDef.Position.Set(5.0f, 6.0f);

                var body6 = World.CreateBody(circleBodyDef);
                body6.CreateFixture(circleShapeDef);
            }
        }
    }
}