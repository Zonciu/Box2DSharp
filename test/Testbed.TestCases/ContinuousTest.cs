using System.Numerics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Continuous","Continuous Test")]
    public class ContinuousTest : TestBase
    {
        private float _angularVelocity;

        private Body _body;

        private GJkProfile _gJkProfile = new GJkProfile();

        private ToiProfile _toiProfile = new ToiProfile();

        public ContinuousTest()
        {
            {
                World.ToiProfile = _toiProfile;
                World.GJkProfile = _gJkProfile;
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
                bd.Position.Set(0.0f, 20.0f);

                //bd.angle = 0.1f;

                var shape = new PolygonShape();
                shape.SetAsBox(2.0f, 0.1f);

                _body = World.CreateBody(bd);
                _body.CreateFixture(shape, 1.0f);

                _angularVelocity = RandomFloat(-50.0f, 50.0f);

                //m_angularVelocity = 46.661274f;
                _body.SetLinearVelocity(new Vector2(0.0f, -100.0f));
                _body.SetAngularVelocity(_angularVelocity);
            }
        }

        private void Launch()
        {
            _gJkProfile.GjkCalls = 0;
            _gJkProfile.GjkIters = 0;
            _gJkProfile.GjkMaxIters = 0;

            _toiProfile.ToiCalls = 0;
            _toiProfile.ToiIters = 0;

            _toiProfile.ToiRootIters = 0;
            _toiProfile.ToiMaxRootIters = 0;

            _toiProfile.ToiTime = 0.0f;
            _toiProfile.ToiMaxTime = 0.0f;

            _body.SetTransform(new Vector2(0.0f, 20.0f), 0.0f);

            _angularVelocity = RandomFloat(-50.0f, 50.0f);

            _body.SetLinearVelocity(new Vector2(0.0f, -100.0f));

            _body.SetAngularVelocity(_angularVelocity);
        }

        protected override void PreStep()
        {
            if (StepCount % 60 == 0)
            {
                Launch();
            }
        }

        /// <inheritdoc />
        protected override void OnRender()
        {
            if (_gJkProfile.GjkCalls > 0)
            {
                DrawString(
                    $"gjk calls = {_gJkProfile.GjkCalls}, ave gjk iters = {_gJkProfile.GjkIters / (float) _gJkProfile.GjkCalls}, max gjk iters = {_gJkProfile.GjkMaxIters}"
                );
            }

            if (_toiProfile.ToiCalls > 0)
            {
                DrawString(
                    $"toi calls = {_toiProfile.ToiCalls}, ave [max] toi iters = {_toiProfile.ToiIters / (float) _toiProfile.ToiCalls} [{_toiProfile.ToiMaxRootIters}]");

                DrawString($"ave [max] toi root iters = {_toiProfile.ToiRootIters / (float) _toiProfile.ToiCalls} [ToiMaxRootIters]");
                DrawString(
                    $"ave [max] toi time = {1000.0f * _toiProfile.ToiTime / (float) _toiProfile.ToiCalls} [{1000.0f * _toiProfile.ToiMaxTime}] (microseconds)");
            }
        }
    }
}