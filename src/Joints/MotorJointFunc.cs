using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public class MotorJointFunc
    {
        public static void SetLinearOffset(JointId jointId, Vec2 linearOffset)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.MotorJoint);
            joint.Joint.MotorJoint.LinearOffset = linearOffset;
        }

        public static Vec2 GetLinearOffset(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.MotorJoint);
            return joint.Joint.MotorJoint.LinearOffset;
        }

        public static void SetAngularOffset(JointId jointId, float angularOffset)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.MotorJoint);
            joint.Joint.MotorJoint.AngularOffset = Math.Clamp(angularOffset, -B2Math.Pi, B2Math.Pi);
        }

        public static float GetAngularOffset(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.MotorJoint);
            return joint.Joint.MotorJoint.AngularOffset;
        }

        public static void SetMaxForce(JointId jointId, float maxForce)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.MotorJoint);
            joint.Joint.MotorJoint.MaxForce = Math.Max(0.0f, maxForce);
        }

        public static float GetMaxForce(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.MotorJoint);
            return joint.Joint.MotorJoint.MaxForce;
        }

        public static void SetMaxTorque(JointId jointId, float maxTorque)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.MotorJoint);
            joint.Joint.MotorJoint.MaxTorque = Math.Max(0.0f, maxTorque);
        }

        public static float GetMaxTorque(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.MotorJoint);
            return joint.Joint.MotorJoint.MaxTorque;
        }

        public static void SetCorrectionFactor(JointId jointId, float correctionFactor)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.MotorJoint);
            joint.Joint.MotorJoint.CorrectionFactor = Math.Clamp(correctionFactor, 0.0f, 1.0f);
        }

        public static float GetCorrectionFactor(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.MotorJoint);
            return joint.Joint.MotorJoint.CorrectionFactor;
        }

        public static Vec2 GetMotorJointForce(World world, JointSim baseSim)
        {
            Vec2 force = B2Math.MulSV(world.InvH, baseSim.Joint.MotorJoint.LinearImpulse);
            return force;
        }

        public static float GetMotorJointTorque(World world, JointSim baseSim)
        {
            return world.InvH * baseSim.Joint.MotorJoint.AngularImpulse;
        }

        // Point-to-point constraint
        // C = p2 - p1
        // Cdot = v2 - v1
        //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
        // J = [-I -r1_skew I r2_skew ]
        // Identity used:
        // w k % (rx i + ry j) = w * (-ry i + rx j)

        // Angle constraint
        // C = angle2 - angle1 - referenceAngle
        // Cdot = w2 - w1
        // J = [0 0 -1 0 0 1]
        // K = invI1 + invI2

        public static void PrepareMotorJoint(JointSim baseSim, StepContext context)
        {
            Debug.Assert(baseSim.Type == JointType.MotorJoint);

            // chase body id to the solver set where the body lives
            int idA = baseSim.BodyIdA;
            int idB = baseSim.BodyIdB;

            World world = context.World;
            var bodies = world.BodyArray;

            bodies.CheckIndex(idA);
            bodies.CheckIndex(idB);

            Body bodyA = bodies[idA];
            Body bodyB = bodies[idB];

            Debug.Assert(bodyA.SetIndex == SolverSetType.AwakeSet || bodyB.SetIndex == SolverSetType.AwakeSet);
            world.SolverSetArray.CheckIndex(bodyA.SetIndex);
            world.SolverSetArray.CheckIndex(bodyB.SetIndex);

            SolverSet setA = world.SolverSetArray[bodyA.SetIndex];
            SolverSet setB = world.SolverSetArray[bodyB.SetIndex];

            int localIndexA = bodyA.LocalIndex;
            int localIndexB = bodyB.LocalIndex;

            Debug.Assert(0 <= localIndexA && localIndexA <= setA.Sims.Count);
            Debug.Assert(0 <= localIndexB && localIndexB <= setB.Sims.Count);

            BodySim bodySimA = setA.Sims.Data[bodyA.LocalIndex];
            BodySim bodySimB = setB.Sims.Data[bodyB.LocalIndex];

            float mA = bodySimA.InvMass;
            float iA = bodySimA.InvInertia;
            float mB = bodySimB.InvMass;
            float iB = bodySimB.InvInertia;

            baseSim.InvMassA = mA;
            baseSim.InvMassB = mB;
            baseSim.InvIA = iA;
            baseSim.InvIB = iB;

            MotorJoint joint = baseSim.Joint.MotorJoint;
            joint.IndexA = bodyA.SetIndex == SolverSetType.AwakeSet ? localIndexA : Core.NullIndex;
            joint.IndexB = bodyB.SetIndex == SolverSetType.AwakeSet ? localIndexB : Core.NullIndex;

            joint.AnchorA = B2Math.RotateVector(bodySimA.Transform.Q, B2Math.Sub(baseSim.LocalOriginAnchorA, bodySimA.LocalCenter));
            joint.AnchorB = B2Math.RotateVector(bodySimB.Transform.Q, B2Math.Sub(baseSim.LocalOriginAnchorB, bodySimB.LocalCenter));
            joint.DeltaCenter = B2Math.Sub(B2Math.Sub(bodySimB.Center, bodySimA.Center), joint.LinearOffset);
            joint.DeltaAngle = B2Math.RelativeAngle(bodySimB.Transform.Q, bodySimA.Transform.Q) - joint.AngularOffset;
            joint.DeltaAngle = B2Math.UnwindAngle(joint.DeltaAngle);

            Vec2 rA = joint.AnchorA;
            Vec2 rB = joint.AnchorB;

            Mat22 K;
            K.Cx.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
            K.Cx.Y = -rA.Y * rA.X * iA - rB.Y * rB.X * iB;
            K.Cy.X = K.Cx.Y;
            K.Cy.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
            joint.LinearMass = B2Math.GetInverse22(K);

            float ka = iA + iB;
            joint.AngularMass = ka > 0.0f ? 1.0f / ka : 0.0f;

            if (context.EnableWarmStarting == false)
            {
                joint.LinearImpulse = Vec2.Zero;
                joint.AngularImpulse = 0.0f;
            }
        }

        public static void WarmStartMotorJoint(JointSim baseSim, StepContext context)
        {
            float mA = baseSim.InvMassA;
            float mB = baseSim.InvMassB;
            float iA = baseSim.InvIA;
            float iB = baseSim.InvIB;

            MotorJoint joint = baseSim.Joint.MotorJoint;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            ref BodyState bodyA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState bodyB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 rA = B2Math.RotateVector(bodyA.DeltaRotation, joint.AnchorA);
            Vec2 rB = B2Math.RotateVector(bodyB.DeltaRotation, joint.AnchorB);

            bodyA.LinearVelocity = B2Math.MulSub(bodyA.LinearVelocity, mA, joint.LinearImpulse);
            bodyA.AngularVelocity -= iA * (B2Math.Cross(rA, joint.LinearImpulse) + joint.AngularImpulse);
            bodyB.LinearVelocity = B2Math.MulAdd(bodyB.LinearVelocity, mB, joint.LinearImpulse);
            bodyB.AngularVelocity += iB * (B2Math.Cross(rB, joint.LinearImpulse) + joint.AngularImpulse);
        }

        public static void SolveMotorJoint(JointSim baseSim, StepContext context, bool useBias)
        {
            Debug.Assert(baseSim.Type == JointType.MotorJoint);

            float mA = baseSim.InvMassA;
            float mB = baseSim.InvMassB;
            float iA = baseSim.InvIA;
            float iB = baseSim.InvIB;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            MotorJoint joint = baseSim.Joint.MotorJoint;
            ref BodyState bodyA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState bodyB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 vA = bodyA.LinearVelocity;
            float wA = bodyA.AngularVelocity;
            Vec2 vB = bodyB.LinearVelocity;
            float wB = bodyB.AngularVelocity;

            // angular constraint
            {
                float angularSeperation = B2Math.RelativeAngle(bodyB.DeltaRotation, bodyA.DeltaRotation) + joint.DeltaAngle;
                angularSeperation = B2Math.UnwindAngle(angularSeperation);

                float angularBias = context.InvH * joint.CorrectionFactor * angularSeperation;

                float Cdot = wB - wA;
                float impulse = -joint.AngularMass * (Cdot + angularBias);

                float oldImpulse = joint.AngularImpulse;
                float maxImpulse = context.H * joint.MaxTorque;
                joint.AngularImpulse = Math.Clamp(joint.AngularImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = joint.AngularImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // linear constraint
            {
                Vec2 rA = B2Math.RotateVector(bodyA.DeltaRotation, joint.AnchorA);
                Vec2 rB = B2Math.RotateVector(bodyB.DeltaRotation, joint.AnchorB);

                Vec2 ds = B2Math.Add(B2Math.Sub(bodyB.DeltaPosition, bodyA.DeltaPosition), B2Math.Sub(rB, rA));
                Vec2 linearSeparation = B2Math.Add(joint.DeltaCenter, ds);
                Vec2 linearBias = B2Math.MulSV(context.InvH * joint.CorrectionFactor, linearSeparation);

                Vec2 Cdot = B2Math.Sub(B2Math.Add(vB, B2Math.CrossSV(wB, rB)), B2Math.Add(vA, B2Math.CrossSV(wA, rA)));
                Vec2 b = B2Math.MulMV(joint.LinearMass, B2Math.Add(Cdot, linearBias));
                Vec2 impulse = new(-b.X, -b.Y);

                Vec2 oldImpulse = joint.LinearImpulse;
                float maxImpulse = context.H * joint.MaxForce;
                joint.LinearImpulse = B2Math.Add(joint.LinearImpulse, impulse);

                if (B2Math.LengthSquared(joint.LinearImpulse) > maxImpulse * maxImpulse)
                {
                    joint.LinearImpulse = joint.LinearImpulse.Normalize;
                    joint.LinearImpulse.X *= maxImpulse;
                    joint.LinearImpulse.Y *= maxImpulse;
                }

                impulse = B2Math.Sub(joint.LinearImpulse, oldImpulse);

                vA = B2Math.MulSub(vA, mA, impulse);
                wA -= iA * B2Math.Cross(rA, impulse);
                vB = B2Math.MulAdd(vB, mB, impulse);
                wB += iB * B2Math.Cross(rB, impulse);
            }

            bodyA.LinearVelocity = vA;
            bodyA.AngularVelocity = wA;
            bodyB.LinearVelocity = vB;
            bodyB.AngularVelocity = wB;
        }
    }
}