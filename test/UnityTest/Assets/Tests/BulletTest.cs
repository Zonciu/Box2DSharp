using System.Numerics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;

namespace Box2DSharp.Tests
{
    public class BulletTest : TestBase
    {
        private Body _body;

        private Body _bullet;

        private float _x;

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

        public int ToiMaxIters
        {
            get => TimeOfImpact.ToiMaxIters;
            set => TimeOfImpact.ToiMaxIters = value;
        }

        protected override void Create()
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

            GjkCalls = 0;
            GjkIters = 0;
            GjkMaxIters = 0;

            ToiCalls = 0;
            ToiIters = 0;
            ToiMaxIters = 0;
            ToiRootIters = 0;
            ToiMaxRootIters = 0;
        }

        /// <inheritdoc />
        protected override void PostStep()
        {
            if (GjkCalls > 0)
            {
                DrawString($"gjk calls = {GjkCalls}, ave gjk iters = {GjkIters / (float) GjkCalls}, max gjk iters = {GjkMaxIters}");
            }

            if (ToiCalls > 0)
            {
                DrawString($"toi calls = {ToiCalls}, ave toi iters = {ToiIters / (float) ToiCalls}, max toi iters = {ToiMaxRootIters}");
                DrawString($"ave toi root iters = {ToiRootIters / (float) ToiCalls}, max toi root iters = {ToiMaxRootIters}");
            }

            if (FrameManager.FrameCount % 60 == 0)
            {
                Launch();
            }
        }
    }
}