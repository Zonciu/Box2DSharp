using System;
using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using Box2DSharp.Dynamics.Joints;
using Shouldly;
using Xunit;

namespace UnitTest
{
    public class JointTest
    {
        [Fact(DisplayName = "joint reactions")]
        public void JointReactions()
        {
            var gravity = new Vector2(0, -10.0f);
            World world = new World(gravity);

            BodyDef bodyDef = new BodyDef();
            Body ground = world.CreateBody(bodyDef);

            CircleShape circle = new CircleShape();
            circle.Radius = 1.0f;

            FixtureDef fixtureDef = new FixtureDef();

            // Disable collision
            fixtureDef.Filter.MaskBits = 0;
            fixtureDef.Density = 1.0f;
            fixtureDef.Shape = circle;

            bodyDef.BodyType = BodyType.DynamicBody;
            bodyDef.Position.Set(-2.0f, 3.0f);

            var bodyA = world.CreateBody(bodyDef);
            var bodyB = world.CreateBody(bodyDef);
            var bodyC = world.CreateBody(bodyDef);

            circle.ComputeMass(out var massData, fixtureDef.Density);
            var mg = massData.Mass * gravity.Y;

            bodyA.CreateFixture(fixtureDef);
            bodyB.CreateFixture(fixtureDef);
            bodyC.CreateFixture(fixtureDef);

            DistanceJointDef distanceJointDef = new DistanceJointDef();
            distanceJointDef.Initialize(ground, bodyA, bodyDef.Position + new Vector2(0.0f, 4.0f), bodyDef.Position);
            distanceJointDef.MinLength = distanceJointDef.Length;
            distanceJointDef.MaxLength = distanceJointDef.Length;

            PrismaticJointDef prismaticJointDef = new PrismaticJointDef();
            prismaticJointDef.Initialize(ground, bodyB, bodyDef.Position, new Vector2(1.0f, 0.0f));

            RevoluteJointDef revoluteJointDef = new RevoluteJointDef();
            revoluteJointDef.Initialize(ground, bodyC, bodyDef.Position);

            var distanceJoint = (DistanceJoint)world.CreateJoint(distanceJointDef);
            var prismaticJoint = (PrismaticJoint)world.CreateJoint(prismaticJointDef);
            var revoluteJoint = (RevoluteJoint)world.CreateJoint(revoluteJointDef);

            const float timeStep = 1 / 60f;
            const float invTimeStep = 60.0f;
            const int velocityIterations = 6;
            const int positionIterations = 2;

            world.Step(timeStep, velocityIterations, positionIterations);

            const float tol = 1e-5f;
            {
                var F = distanceJoint.GetReactionForce(invTimeStep);
                var T = distanceJoint.GetReactionTorque(invTimeStep);
                F.X.ShouldBe(0.0f);
                Math.Abs(F.Y + mg).ShouldBeLessThan(tol);
                T.ShouldBe(0.0f);
            }

            {
                var F = prismaticJoint.GetReactionForce(invTimeStep);
                var T = prismaticJoint.GetReactionTorque(invTimeStep);
                F.X.ShouldBe(0.0f);
                Math.Abs(F.Y + mg).ShouldBeLessThan(tol);
                T.ShouldBe(0.0f);
            }

            {
                var F = revoluteJoint.GetReactionForce(invTimeStep);
                var T = revoluteJoint.GetReactionTorque(invTimeStep);
                F.X.ShouldBe(0.0f);
                Math.Abs(F.Y + mg).ShouldBeLessThan(tol);
                T.ShouldBe(0.0f);
            }
        }
    }
}