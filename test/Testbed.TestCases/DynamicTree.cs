using System;
using System.Diagnostics;
using Box2DSharp.Collision;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Common;
using Box2DSharp.Dynamics.Internal;
using Testbed.Abstractions;
using Color = Box2DSharp.Common.Color;
using Random = System.Random;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.TestCases
{
    [TestCase("Collision", "Dynamic Tree")]
    public class DynamicTree : TestBase, ITreeQueryCallback, ITreeRayCastCallback
    {
        private const int ActorCount = 128;

        private readonly Random _random = new Random(888);

        private readonly Actor[] _actors = new Actor[ActorCount].Fill();

        private bool _automated;

        private float _proxyExtent;

        private AABB _queryAABB;

        private Actor _rayActor = new Actor();

        private RayCastInput _rayCastInput;

        private RayCastOutput _rayCastOutput;

        private readonly Box2DSharp.Collision.DynamicTree _tree = new Box2DSharp.Collision.DynamicTree();

        private float _worldExtent;

        public DynamicTree()
        {
            _worldExtent = 15.0f;
            _proxyExtent = 0.5f;

            for (var i = 0; i < ActorCount; ++i)
            {
                var actor = _actors[i];
                GetRandomAABB(ref actor.AABB);
                actor.ProxyId = _tree.CreateProxy(actor.AABB, actor);
            }

            var h = _worldExtent;
            _queryAABB.LowerBound.Set(-3.0f, -4.0f + h);
            _queryAABB.UpperBound.Set(5.0f, 6.0f + h);

            _rayCastInput.P1.Set(-5.0f, 5.0f + h);
            _rayCastInput.P2.Set(7.0f, -4.0f + h);

            _rayCastInput.MaxFraction = 1.0f;

            _automated = false;
        }

        /// <inheritdoc />
        public override void OnKeyDown(KeyInputEventArgs keyInput)
        {
            if (keyInput.Key == KeyCodes.A)
            {
                _automated = !_automated;
            }

            if (keyInput.Key == KeyCodes.C)
            {
                CreateProxy();
            }

            if (keyInput.Key == KeyCodes.D)
            {
                DestroyProxy();
            }

            if (keyInput.Key == KeyCodes.M)
            {
                MoveProxy();
            }
        }

        protected override void PreStep()
        {
            _rayActor = null;
            for (var i = 0; i < ActorCount; ++i)
            {
                _actors[i].Fraction = 1.0f;
                _actors[i].Overlap = false;
            }

            if (_automated)
            {
                var actionCount = Math.Max(1, ActorCount >> 2);

                for (var i = 0; i < actionCount; ++i)
                {
                    Action();
                }
            }

            Query();
            RayCast();
        }

        protected override void OnRender()
        {
            DrawString("Keys: a: automate, c: create, d: destroy, m: move");
            DrawString("Blue: overlap");
            DrawString("Green: ray actor");
            DrawString("Red: ray actor & overlap");
            Color c;
            for (var i = 0; i < ActorCount; ++i)
            {
                var actor = _actors[i];
                if (actor.ProxyId == BroadPhase.NullProxy)
                {
                    continue;
                }

                c = Color.FromArgb(0.9f, 0.9f, 0.9f);
                if (actor == _rayActor && actor.Overlap)
                {
                    c = Color.FromArgb(0.9f, 0.6f, 0.6f);
                }
                else if (actor == _rayActor)
                {
                    c = Color.FromArgb(0.6f, 0.9f, 0.6f);
                }
                else if (actor.Overlap)
                {
                    c = Color.FromArgb(0.6f, 0.6f, 0.9f);
                }

                Drawer.DrawAABB(actor.AABB, c);
            }

            c = Color.FromArgb(0.7f, 0.7f, 0.7f);
            Drawer.DrawAABB(_queryAABB, c);

            Drawer.DrawSegment(_rayCastInput.P1, _rayCastInput.P2, c);

            var c1 = Color.FromArgb(0.2f, 0.9f, 0.2f);
            var c2 = Color.FromArgb(0.9f, 0.2f, 0.2f);
            Drawer.DrawPoint(_rayCastInput.P1, 6.0f, c1);
            Drawer.DrawPoint(_rayCastInput.P2, 6.0f, c2);

            if (_rayActor != null)
            {
                var cr = Color.FromArgb(0.2f, 0.2f, 0.9f);
                var p = _rayCastInput.P1 + _rayActor.Fraction * (_rayCastInput.P2 - _rayCastInput.P1);
                Drawer.DrawPoint(p, 6.0f, cr);
            }

            {
                var height = _tree.GetHeight();
                DrawString($"dynamic tree height = {height}");
            }
        }

        // public void DrawAABB(AABB aabb, Color color)
        // {
        //     var vs = new Vector2 [4];
        //     vs[0].Set(aabb.LowerBound.X, aabb.LowerBound.Y);
        //     vs[1].Set(aabb.UpperBound.X, aabb.LowerBound.Y);
        //     vs[2].Set(aabb.UpperBound.X, aabb.UpperBound.Y);
        //     vs[3].Set(aabb.LowerBound.X, aabb.UpperBound.Y);
        //
        //     Drawer.DrawPolygon(vs, 4, color);
        // }

        public bool QueryCallback(int proxyId)
        {
            var actor = (Actor)_tree.GetUserData(proxyId);
            actor.Overlap = CollisionUtils.TestOverlap(_queryAABB, actor.AABB);
            return true;
        }

        public float RayCastCallback(in RayCastInput input, int proxyId)
        {
            var actor = (Actor)_tree.GetUserData(proxyId);
            var hit = actor.AABB.RayCast(out var output, input);

            if (hit)
            {
                _rayCastOutput = output;
                _rayActor = actor;
                _rayActor.Fraction = output.Fraction;
                return output.Fraction;
            }

            return input.MaxFraction;
        }

        private void GetRandomAABB(ref AABB aabb)
        {
            var w = new Vector2(2.0f * _proxyExtent, 2.0f * _proxyExtent);

            //aabb.LowerBound.X = -proxyExtent;
            //aabb.LowerBound.Y = -proxyExtent + worldExtent;
            aabb.LowerBound.X = RandomFloat(-_worldExtent, _worldExtent);
            aabb.LowerBound.Y = RandomFloat(0.0f, 2.0f * _worldExtent);
            aabb.UpperBound = aabb.LowerBound + w;
        }

        private void MoveAABB(ref AABB aabb)
        {
            Vector2 d;
            d.X = RandomFloat(-0.5f, 0.5f);
            d.Y = RandomFloat(-0.5f, 0.5f);

            //d.X = 2.0f;
            //d.Y = 0.0f;
            aabb.LowerBound += d;
            aabb.UpperBound += d;

            var c0 = 0.5f * (aabb.LowerBound + aabb.UpperBound);
            var min = new Vector2();
            min.Set(-_worldExtent, 0.0f);
            var max = new Vector2();
            max.Set(_worldExtent, 2.0f * _worldExtent);
            var c = Vector2.Clamp(c0, min, max);

            aabb.LowerBound += c - c0;
            aabb.UpperBound += c - c0;
        }

        private void CreateProxy()
        {
            for (var i = 0; i < ActorCount; ++i)
            {
                var j = _random.Next(0, ActorCount);
                var actor = _actors[j];
                if (actor.ProxyId == BroadPhase.NullProxy)
                {
                    GetRandomAABB(ref actor.AABB);
                    actor.ProxyId = _tree.CreateProxy(actor.AABB, actor);
                    return;
                }
            }
        }

        private void DestroyProxy()
        {
            for (var i = 0; i < ActorCount; ++i)
            {
                var j = _random.Next(0, ActorCount);
                var actor = _actors[j];
                if (actor.ProxyId != BroadPhase.NullProxy)
                {
                    _tree.DestroyProxy(actor.ProxyId);
                    actor.ProxyId = BroadPhase.NullProxy;
                    return;
                }
            }
        }

        private void MoveProxy()
        {
            for (var i = 0; i < ActorCount; ++i)
            {
                var j = _random.Next(0, ActorCount);
                var actor = _actors[j];
                if (actor.ProxyId == BroadPhase.NullProxy)
                {
                    continue;
                }

                var aabb0 = actor.AABB;
                MoveAABB(ref actor.AABB);
                var displacement = actor.AABB.GetCenter() - aabb0.GetCenter();
                _tree.MoveProxy(actor.ProxyId, actor.AABB, displacement);
                return;
            }
        }

        private void Action()
        {
            var choice = _random.Next(0, 20);

            switch (choice)
            {
            case 0:
                CreateProxy();
                break;

            case 1:
                DestroyProxy();
                break;

            default:
                MoveProxy();
                break;
            }
        }

        private void Query()
        {
            _tree.Query(this, _queryAABB);

            for (var i = 0; i < ActorCount; ++i)
            {
                if (_actors[i].ProxyId == BroadPhase.NullProxy)
                {
                    continue;
                }

                var overlap = CollisionUtils.TestOverlap(_queryAABB, _actors[i].AABB);

                Debug.Assert(overlap == _actors[i].Overlap);
            }
        }

        private void RayCast()
        {
            _rayActor = null;

            var input = _rayCastInput;

            // Ray cast against the dynamic tree.
            _tree.RayCast(this, input);

            // Brute force ray cast.
            Actor bruteActor = null;
            var bruteOutput = new RayCastOutput();
            for (var i = 0; i < ActorCount; ++i)
            {
                if (_actors[i].ProxyId == BroadPhase.NullProxy)
                {
                    continue;
                }

                var hit = _actors[i].AABB.RayCast(out var output, input);
                if (hit)
                {
                    bruteActor = _actors[i];
                    bruteOutput = output;
                    input.MaxFraction = output.Fraction;
                }
            }

            if (bruteActor != null)
            {
                Debug.Assert(Math.Abs(bruteOutput.Fraction - _rayCastOutput.Fraction) < Settings.Epsilon);
            }
        }

        public class Actor
        {
            public AABB AABB;

            public float Fraction;

            public bool Overlap;

            public int ProxyId = BroadPhase.NullProxy;
        }
    }
}