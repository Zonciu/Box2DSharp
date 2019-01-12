using System.Numerics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;

namespace Box2DSharp.Tests
{
    public class ContinuousTest : TestBase
    {
        private float _angularVelocity;

        private Body _body;

        public int GjkCalls
        {
            get => DistanceAlgorithm.GjkCalls;
            set => DistanceAlgorithm.GjkCalls = value;
        }

        public int GjkIters
        {
            get => DistanceAlgorithm.GjkIters;
            set => DistanceAlgorithm.GjkIters = value;
        }

        public int GjkMaxIters
        {
            get => DistanceAlgorithm.GjkMaxIters;
            set => DistanceAlgorithm.GjkMaxIters = value;
        }

        public int ToiCalls
        {
            get => TimeOfImpact.ToiCalls;
            set => TimeOfImpact.ToiCalls = value;
        }

        public int ToiIters
        {
            get => TimeOfImpact.ToiIters;
            set => TimeOfImpact.ToiIters = value;
        }

        public int ToiRootIters
        {
            get => TimeOfImpact.ToiRootIters;
            set => TimeOfImpact.ToiRootIters = value;
        }

        public int ToiMaxRootIters
        {
            get => TimeOfImpact.ToiMaxRootIters;
            set => TimeOfImpact.ToiMaxRootIters = value;
        }

        public float ToiTime
        {
            get => TimeOfImpact.ToiTime;
            set => TimeOfImpact.ToiTime = value;
        }

        public float ToiMaxTime
        {
            get => TimeOfImpact.ToiMaxTime;
            set => TimeOfImpact.ToiMaxTime = value;
        }

        private void Start()
        {
            {
                var bd = new BodyDef();
                bd.Position.Set(0.0f, 0.0f);
                var body = World.CreateBody(bd);

                var edge = new EdgeShape();

                edge.Set(new Vector2(-10.0f, 0.0f), new Vector2(10.0f, 0.0f));
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

            GjkCalls = 0;
            GjkIters = 0;
            GjkMaxIters = 0;
            ToiCalls = 0;
            ToiIters = 0;
            ToiRootIters = 0;
            ToiMaxRootIters = 0;
            ToiTime = 0.0f;
            ToiMaxTime = 0.0f;
        }

        private void Launch()
        {
            GjkCalls = 0;
            GjkIters = 0;
            GjkMaxIters = 0;

            ToiCalls = 0;
            ToiIters = 0;

            ToiRootIters = 0;
            ToiMaxRootIters = 0;

            ToiTime = 0.0f;
            ToiMaxTime = 0.0f;

            _body.SetTransform(new Vector2(0.0f, 20.0f), 0.0f);

            _angularVelocity = RandomFloat(-50.0f, 50.0f);

            _body.SetLinearVelocity(new Vector2(0.0f, -100.0f));

            _body.SetAngularVelocity(_angularVelocity);
        }

        /// <inheritdoc />
        protected override void PostStep()
        {
            if (GjkCalls > 0)
            {
                DrawString(
                    $"gjk calls = {GjkCalls}, ave gjk iters = {GjkIters / (float) GjkCalls}, max gjk iters = {GjkMaxIters}"
                );
            }

            if (ToiCalls > 0)
            {
                DrawString(
                    $"toi calls = {ToiCalls}, ave [max] toi iters = {ToiIters / (float) ToiCalls} [{ToiMaxRootIters}]");

                DrawString($"ave [max] toi root iters = {ToiRootIters / (float) ToiCalls} [ToiMaxRootIters]");
                DrawString(
                    $"ave [max] toi time = {1000.0f * ToiTime / (float) ToiCalls} [{1000.0f * ToiMaxTime}] (microseconds)");
            }
        }
    }
}