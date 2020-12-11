using System;
using System.Collections.Generic;
using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Testbed.Abstractions;

namespace Testbed.TestCases
{
    [TestCase("Examples", "Collision Processing")]
    public class CollisionProcessing : TestBase
    {
        public CollisionProcessing()
        {
            // Ground body
            {
                var shape = new EdgeShape();
                shape.SetTwoSided(new Vector2(-50.0f, 0.0f), new Vector2(50.0f, 0.0f));

                var sd = new FixtureDef();
                sd.Shape = shape;
                ;

                var bd = new BodyDef();
                var ground = World.CreateBody(bd);
                ground.CreateFixture(sd);
            }

            float xLo = -5.0f, xHi = 5.0f;
            float yLo = 2.0f, yHi = 35.0f;

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

            var triangleBodyDef = new BodyDef();
            triangleBodyDef.BodyType = BodyType.DynamicBody;
            triangleBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

            var body1 = World.CreateBody(triangleBodyDef);
            body1.CreateFixture(triangleShapeDef);

            // Large triangle (recycle definitions)
            vertices[0] *= 2.0f;
            vertices[1] *= 2.0f;
            vertices[2] *= 2.0f;
            polygon.Set(vertices);

            triangleBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

            var body2 = World.CreateBody(triangleBodyDef);
            body2.CreateFixture(triangleShapeDef);

            // Small box
            polygon.SetAsBox(1.0f, 0.5f);

            var boxShapeDef = new FixtureDef();
            boxShapeDef.Shape = polygon;
            boxShapeDef.Density = 1.0f;

            var boxBodyDef = new BodyDef();
            boxBodyDef.BodyType = BodyType.DynamicBody;
            boxBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

            var body3 = World.CreateBody(boxBodyDef);
            body3.CreateFixture(boxShapeDef);

            // Large box (recycle definitions)
            polygon.SetAsBox(2.0f, 1.0f);
            boxBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

            var body4 = World.CreateBody(boxBodyDef);
            body4.CreateFixture(boxShapeDef);

            // Small circle
            var circle = new CircleShape();
            circle.Radius = 1.0f;

            var circleShapeDef = new FixtureDef();
            circleShapeDef.Shape = circle;
            circleShapeDef.Density = 1.0f;

            var circleBodyDef = new BodyDef();
            circleBodyDef.BodyType = BodyType.DynamicBody;
            circleBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

            var body5 = World.CreateBody(circleBodyDef);
            body5.CreateFixture(circleShapeDef);

            // Large circle
            circle.Radius *= 2.0f;
            circleBodyDef.Position.Set(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

            var body6 = World.CreateBody(circleBodyDef);
            body6.CreateFixture(circleShapeDef);
        }

        /// <inheritdoc />
        protected override void PostStep()
        {
            // We are going to destroy some bodies according to contact
            // points. We must buffer the bodies that should be destroyed
            // because they may belong to multiple contact points.
            const int maxNuke = 6;
            var nuke = new Body[maxNuke];
            var nukeCount = 0;

            // Traverse the contact results. Destroy bodies that
            // are touching heavier bodies.
            for (var i = 0; i < PointsCount; ++i)
            {
                var point = Points[i];

                var body1 = point.FixtureA.Body;
                var body2 = point.FixtureB.Body;
                var mass1 = body1.Mass;
                var mass2 = body2.Mass;

                if (mass1 > 0.0f && mass2 > 0.0f)
                {
                    if (mass2 > mass1)
                    {
                        nuke[nukeCount++] = body1;
                    }
                    else
                    {
                        nuke[nukeCount++] = body2;
                    }

                    if (nukeCount == maxNuke)
                    {
                        break;
                    }
                }
            }

            // Sort the nuke array to group duplicates.
            Array.Sort(nuke, 0, nukeCount, new BodyComparer());

            // Destroy the bodies, skipping duplicates.
            {
                var i = 0;
                while (i < nukeCount)
                {
                    var b = nuke[i++];
                    while (i < nukeCount && nuke[i] == b)
                    {
                        ++i;
                    }

                    if (b != Bomb)
                    {
                        World.DestroyBody(b);
                    }
                }
            }
        }
    }

    struct BodyComparer : IComparer<Body>
    {
        /// <inheritdoc />
        public int Compare(Body x, Body y)
        {
            return x.GetHashCode() - y.GetHashCode();
        }
    }
}