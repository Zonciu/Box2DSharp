using System.Diagnostics;

namespace Box2DSharp
{
    public class MouseJointFunc
    {
        public static void SetTarget(JointId jointId, Vec2 target)
        {
            Debug.Assert(B2Math.Vec2_IsValid(target));
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.MouseJoint);
            baseSim.Joint.MouseJoint.TargetA = target;
        }

        public static Vec2 GetTarget(JointId jointId)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.MouseJoint);
            return baseSim.Joint.MouseJoint.TargetA;
        }

        public static void SetSpringHertz(JointId jointId, float hertz)
        {
            Debug.Assert(B2Math.IsValid(hertz) && hertz >= 0.0f);
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.MouseJoint);
            baseSim.Joint.MouseJoint.Hertz = hertz;
        }

        public static float GetSpringHertz(JointId jointId)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.MouseJoint);
            return baseSim.Joint.MouseJoint.Hertz;
        }

        public static void SetSpringDampingRatio(JointId jointId, float dampingRatio)
        {
            Debug.Assert(B2Math.IsValid(dampingRatio) && dampingRatio >= 0.0f);
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.MouseJoint);
            baseSim.Joint.MouseJoint.DampingRatio = dampingRatio;
        }

        public static float GetSpringDampingRatio(JointId jointId)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.MouseJoint);
            return baseSim.Joint.MouseJoint.DampingRatio;
        }

        public static void SetMaxForce(JointId jointId, float maxForce)
        {
            Debug.Assert(B2Math.IsValid(maxForce) && maxForce >= 0.0f);
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.MouseJoint);
            baseSim.Joint.MouseJoint.MaxForce = maxForce;
        }

        public static float GetMaxForce(JointId jointId)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.MouseJoint);
            return baseSim.Joint.MouseJoint.MaxForce;
        }

        public static Vec2 GetMouseJointForce(World world, JointSim baseSim)
        {
            Vec2 force = B2Math.MulSV(world.InvH, baseSim.Joint.MouseJoint.LinearImpulse);
            return force;
        }

        public static float GetMouseJointTorque(World world, JointSim baseSim)
        {
            return world.InvH * baseSim.Joint.MouseJoint.AngularImpulse;
        }

        public static void PrepareMouseJoint(JointSim baseSim, StepContext context)
        {
            Debug.Assert(baseSim.Type == JointType.MouseJoint);

            // chase body id to the solver set where the body lives
            int idB = baseSim.BodyIdB;

            World world = context.World;
            var bodies = world.BodyArray;

            bodies.CheckIndex(idB);

            Body bodyB = bodies[idB];

            Debug.Assert(bodyB.SetIndex == SolverSetType.AwakeSet);
            world.SolverSetArray.CheckIndex(bodyB.SetIndex);

            SolverSet setB = world.SolverSetArray[bodyB.SetIndex];

            int localIndexB = bodyB.LocalIndex;
            Debug.Assert(0 <= localIndexB && localIndexB <= setB.Sims.Count);

            BodySim bodySimB = setB.Sims.Data[localIndexB];

            baseSim.InvMassB = bodySimB.InvMass;
            baseSim.InvIB = bodySimB.InvInertia;

            MouseJoint joint = baseSim.Joint.MouseJoint;
            joint.IndexB = bodyB.SetIndex == SolverSetType.AwakeSet ? localIndexB : Core.NullIndex;
            joint.AnchorB = B2Math.RotateVector(bodySimB.Transform.Q, B2Math.Sub(baseSim.LocalOriginAnchorB, bodySimB.LocalCenter));

            joint.LinearSoftness = Softness.MakeSoft(joint.Hertz, joint.DampingRatio, context.H);

            float angularHertz = 0.5f;
            float angularDampingRatio = 0.1f;
            joint.AngularSoftness = Softness.MakeSoft(angularHertz, angularDampingRatio, context.H);

            Vec2 rB = joint.AnchorB;
            float mB = bodySimB.InvMass;
            float iB = bodySimB.InvInertia;

            // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
            //   = [1/m1+1/m2     0    ] + invI1 * [r1.Y*r1.Y -r1.X*r1.Y] + invI2 * [r1.Y*r1.Y -r1.X*r1.Y]
            //     [    0     1/m1+1/m2]           [-r1.X*r1.Y r1.X*r1.X]           [-r1.X*r1.Y r1.X*r1.X]
            Mat22 K;
            K.Cx.X = mB + iB * rB.Y * rB.Y;
            K.Cx.Y = -iB * rB.X * rB.Y;
            K.Cy.X = K.Cx.Y;
            K.Cy.Y = mB + iB * rB.X * rB.X;

            joint.LinearMass = B2Math.GetInverse22(K);
            joint.DeltaCenter = B2Math.Sub(bodySimB.Center, joint.TargetA);

            if (context.EnableWarmStarting == false)
            {
                joint.LinearImpulse = Vec2.Zero;
                joint.AngularImpulse = 0.0f;
            }
        }

        public static void WarmStartMouseJoint(JointSim baseSim, StepContext context)
        {
            Debug.Assert(baseSim.Type == JointType.MouseJoint);

            float mB = baseSim.InvMassB;
            float iB = baseSim.InvIB;

            MouseJoint joint = baseSim.Joint.MouseJoint;

            ref BodyState stateB = ref context.States[joint.IndexB];
            Vec2 vB = stateB.LinearVelocity;
            float wB = stateB.AngularVelocity;

            Rot dqB = stateB.DeltaRotation;
            Vec2 rB = B2Math.RotateVector(dqB, joint.AnchorB);

            vB = B2Math.MulAdd(vB, mB, joint.LinearImpulse);
            wB += iB * (B2Math.Cross(rB, joint.LinearImpulse) + joint.AngularImpulse);

            stateB.LinearVelocity = vB;
            stateB.AngularVelocity = wB;
        }

        public static void SolveMouseJoint(JointSim baseSim, StepContext context)
        {
            float mB = baseSim.InvMassB;
            float iB = baseSim.InvIB;

            MouseJoint joint = baseSim.Joint.MouseJoint;
            ref BodyState stateB = ref context.States[joint.IndexB];

            Vec2 vB = stateB.LinearVelocity;
            float wB = stateB.AngularVelocity;

            // Softness with no bias to reduce rotation speed
            {
                float massScale = joint.AngularSoftness.MassScale;
                float impulseScale = joint.AngularSoftness.ImpulseScale;

                float impulse = iB > 0.0f ? -wB / iB : 0.0f;
                impulse = massScale * impulse - impulseScale * joint.AngularImpulse;
                joint.AngularImpulse += impulse;

                wB += iB * impulse;
            }

            float maxImpulse = joint.MaxForce * context.H;

            {
                Rot dqB = stateB.DeltaRotation;
                Vec2 rB = B2Math.RotateVector(dqB, joint.AnchorB);
                Vec2 Cdot = B2Math.Add(vB, B2Math.CrossSV(wB, rB));

                Vec2 separation = B2Math.Add(B2Math.Add(stateB.DeltaPosition, rB), joint.DeltaCenter);
                Vec2 bias = B2Math.MulSV(joint.LinearSoftness.BiasRate, separation);

                float massScale = joint.LinearSoftness.MassScale;
                float impulseScale = joint.LinearSoftness.ImpulseScale;

                Vec2 b = B2Math.MulMV(joint.LinearMass, B2Math.Add(Cdot, bias));

                Vec2 impulse;
                impulse.X = -massScale * b.X - impulseScale * joint.LinearImpulse.X;
                impulse.Y = -massScale * b.Y - impulseScale * joint.LinearImpulse.Y;

                Vec2 oldImpulse = joint.LinearImpulse;
                joint.LinearImpulse.X += impulse.X;
                joint.LinearImpulse.Y += impulse.Y;

                float mag = joint.LinearImpulse.Length;
                if (mag > maxImpulse)
                {
                    joint.LinearImpulse = B2Math.MulSV(maxImpulse, joint.LinearImpulse.Normalize);
                }

                impulse.X = joint.LinearImpulse.X - oldImpulse.X;
                impulse.Y = joint.LinearImpulse.Y - oldImpulse.Y;

                vB = B2Math.MulAdd(vB, mB, impulse);
                wB += iB * B2Math.Cross(rB, impulse);
            }

            stateB.LinearVelocity = vB;
            stateB.AngularVelocity = wB;
        }
    }
}