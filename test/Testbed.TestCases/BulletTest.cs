using System.Numerics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Continuous", "Bullet Test")]
    public class BulletTest : TestBase
    {
        private Body _body;

        private Body _bullet;

        private float _x;

        private GJkProfile _gJkProfile = new GJkProfile();

        private ToiProfile _toiProfile = new ToiProfile();

        public BulletTest()
        {
            {
                var bd = new BodyDef();
                bd.Position.Set(0.0f, 0.0f);
                var body = World.CreateBody(bd);

                var edge = new EdgeShape();

                edge.SetTwoSided(new Vector2(-10.0f, 0.0f), new Vector2(10.0f, 0.0f));
                body.CreateFixture(edge, 0.0f);

                var shape = new PolygonShape();
                shape.SetAsBox(0.2f, 1.0f, new Vector2(0.5f, 1.0f), 0.0f);
                body.CreateFixture(shape, 0.0f);
            }

            {
                var bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;
                bd.Position.Set(0.0f, 4.0f);

                var box = new PolygonShape();
                box.SetAsBox(2.0f, 0.1f);

                _body = World.CreateBody(bd);
                _body.CreateFixture(box, 1.0f);

                box.SetAsBox(0.25f, 0.25f);

                //m_x = RandomFloat(-1.0f, 1.0f);
                _x = 0.20352793f;
                bd.Position = new Vector2(_x, 10.0f);
                bd.Bullet = true;

                _bullet = World.CreateBody(bd);
                _bullet.CreateFixture(box, 100.0f);

                _bullet.SetLinearVelocity(new Vector2(0.0f, -50.0f));
            }
        }

        private void Launch()
        {
            _body.SetTransform(new Vector2(0.0f, 4.0f), 0.0f);
            _body.SetLinearVelocity(Vector2.Zero);
            _body.SetAngularVelocity(0.0f);

            _x = RandomFloat(-1.0f, 1.0f);
            _bullet.SetTransform(new Vector2(_x, 10.0f), 0.0f);
            _bullet.SetLinearVelocity(new Vector2(0.0f, -50.0f));
            _bullet.SetAngularVelocity(0.0f);
        }

        protected override void PreStep()
        {
            if (StepCount % 60 == 0)
            {
                Launch();
            }
        }

        protected override void OnRender()
        {
            if (_gJkProfile.GjkCalls > 0)
            {
                DrawString(
                    $"gjk calls = {_gJkProfile.GjkCalls}, ave gjk iters = {_gJkProfile.GjkIters / (float)_gJkProfile.GjkCalls}, max gjk iters = {_gJkProfile.GjkMaxIters}");
            }

            if (_toiProfile.ToiCalls > 0)
            {
                DrawString(
                    $"toi calls = {_toiProfile.ToiCalls}, ave toi iters = {_toiProfile.ToiIters / (float)_toiProfile.ToiCalls}, max toi iters = {_toiProfile.ToiMaxRootIters}");
                DrawString(
                    $"ave toi root iters = {_toiProfile.ToiRootIters / (float)_toiProfile.ToiCalls}, max toi root iters = {_toiProfile.ToiMaxRootIters}");
            }
        }
    }
}