using System.Diagnostics;

namespace Box2DSharp
{
    public class WeldJointFunc
    {
        public static void SetLinearHertz(JointId jointId, float hertz)
        {
            Debug.Assert(B2Math.IsValid(hertz) && hertz >= 0.0f);
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WeldJoint);
            joint.Joint.WeldJoint.LinearHertz = hertz;
        }

        public static float GetLinearHertz(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WeldJoint);
            return joint.Joint.WeldJoint.LinearHertz;
        }

        public static void SetLinearDampingRatio(JointId jointId, float dampingRatio)
        {
            Debug.Assert(B2Math.IsValid(dampingRatio) && dampingRatio >= 0.0f);
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WeldJoint);
            joint.Joint.WeldJoint.LinearDampingRatio = dampingRatio;
        }

        public static float GetLinearDampingRatio(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WeldJoint);
            return joint.Joint.WeldJoint.LinearDampingRatio;
        }

        public static void SetAngularHertz(JointId jointId, float hertz)
        {
            Debug.Assert(B2Math.IsValid(hertz) && hertz >= 0.0f);
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WeldJoint);
            joint.Joint.WeldJoint.AngularHertz = hertz;
        }

        public static float GetAngularHertz(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WeldJoint);
            return joint.Joint.WeldJoint.AngularHertz;
        }

        public static void SetAngularDampingRatio(JointId jointId, float dampingRatio)
        {
            Debug.Assert(B2Math.IsValid(dampingRatio) && dampingRatio >= 0.0f);
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WeldJoint);
            joint.Joint.WeldJoint.AngularDampingRatio = dampingRatio;
        }

        public static float GetAngularDampingRatio(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WeldJoint);
            return joint.Joint.WeldJoint.AngularDampingRatio;
        }

        public static Vec2 GetWeldJointForce(World world, JointSim baseSim)
        {
            Vec2 force = B2Math.MulSV(world.InvH, baseSim.Joint.WeldJoint.LinearImpulse);
            return force;
        }

        public static float GetWeldJointTorque(World world, JointSim baseSim)
        {
            return world.InvH * baseSim.Joint.WeldJoint.AngularImpulse;
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

        public static void PrepareWeldJoint(JointSim baseSim, StepContext context)
        {
            Debug.Assert(baseSim.Type == JointType.WeldJoint);

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

            WeldJoint joint = baseSim.Joint.WeldJoint;
            joint.IndexA = bodyA.SetIndex == SolverSetType.AwakeSet ? localIndexA : Core.NullIndex;
            joint.IndexB = bodyB.SetIndex == SolverSetType.AwakeSet ? localIndexB : Core.NullIndex;

            Rot qA = bodySimA.Transform.Q;
            Rot qB = bodySimB.Transform.Q;

            joint.AnchorA = B2Math.RotateVector(qA, B2Math.Sub(baseSim.LocalOriginAnchorA, bodySimA.LocalCenter));
            joint.AnchorB = B2Math.RotateVector(qB, B2Math.Sub(baseSim.LocalOriginAnchorB, bodySimB.LocalCenter));
            joint.DeltaCenter = B2Math.Sub(bodySimB.Center, bodySimA.Center);
            joint.DeltaAngle = B2Math.RelativeAngle(qB, qA) - joint.ReferenceAngle;

            float ka = iA + iB;
            joint.AxialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

            float h = context.Dt;

            if (joint.LinearHertz == 0.0f)
            {
                joint.LinearSoftness = context.JointSoftness;
            }
            else
            {
                joint.LinearSoftness = Softness.MakeSoft(joint.LinearHertz, joint.LinearDampingRatio, context.H);
            }

            if (joint.AngularHertz == 0.0f)
            {
                joint.AngularSoftness = context.JointSoftness;
            }
            else
            {
                joint.AngularSoftness = Softness.MakeSoft(joint.AngularHertz, joint.AngularDampingRatio, context.H);
            }

            if (context.EnableWarmStarting == false)
            {
                joint.LinearImpulse = Vec2.Zero;
                joint.AngularImpulse = 0.0f;
            }
        }

        public static void WarmStartWeldJoint(JointSim baseSim, StepContext context)
        {
            float mA = baseSim.InvMassA;
            float mB = baseSim.InvMassB;
            float iA = baseSim.InvIA;
            float iB = baseSim.InvIB;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            WeldJoint joint = baseSim.Joint.WeldJoint;

            ref BodyState stateA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState stateB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 rA = B2Math.RotateVector(stateA.DeltaRotation, joint.AnchorA);
            Vec2 rB = B2Math.RotateVector(stateB.DeltaRotation, joint.AnchorB);

            stateA.LinearVelocity = B2Math.MulSub(stateA.LinearVelocity, mA, joint.LinearImpulse);
            stateA.AngularVelocity -= iA * (B2Math.Cross(rA, joint.LinearImpulse) + joint.AngularImpulse);

            stateB.LinearVelocity = B2Math.MulAdd(stateB.LinearVelocity, mB, joint.LinearImpulse);
            stateB.AngularVelocity += iB * (B2Math.Cross(rB, joint.LinearImpulse) + joint.AngularImpulse);
        }

        public static void SolveWeldJoint(JointSim baseSim, StepContext context, bool useBias)
        {
            Debug.Assert(baseSim.Type == JointType.WeldJoint);

            float mA = baseSim.InvMassA;
            float mB = baseSim.InvMassB;
            float iA = baseSim.InvIA;
            float iB = baseSim.InvIB;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            WeldJoint joint = baseSim.Joint.WeldJoint;

            ref BodyState stateA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState stateB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 vA = stateA.LinearVelocity;
            float wA = stateA.AngularVelocity;
            Vec2 vB = stateB.LinearVelocity;
            float wB = stateB.AngularVelocity;

            // angular constraint
            {
                float bias = 0.0f;
                float massScale = 1.0f;
                float impulseScale = 0.0f;
                if (useBias || joint.AngularHertz > 0.0f)
                {
                    float C = B2Math.RelativeAngle(stateB.DeltaRotation, stateA.DeltaRotation) + joint.DeltaAngle;
                    bias = joint.AngularSoftness.BiasRate * C;
                    massScale = joint.AngularSoftness.MassScale;
                    impulseScale = joint.AngularSoftness.ImpulseScale;
                }

                float Cdot = wB - wA;
                float impulse = -massScale * joint.AxialMass * (Cdot + bias) - impulseScale * joint.AngularImpulse;
                joint.AngularImpulse += impulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // linear constraint
            {
                Vec2 rA = B2Math.RotateVector(stateA.DeltaRotation, joint.AnchorA);
                Vec2 rB = B2Math.RotateVector(stateB.DeltaRotation, joint.AnchorB);

                Vec2 bias = Vec2.Zero;
                float massScale = 1.0f;
                float impulseScale = 0.0f;
                if (useBias || joint.LinearHertz > 0.0f)
                {
                    Vec2 dcA = stateA.DeltaPosition;
                    Vec2 dcB = stateB.DeltaPosition;
                    Vec2 C = B2Math.Add(B2Math.Add(B2Math.Sub(dcB, dcA), B2Math.Sub(rB, rA)), joint.DeltaCenter);

                    bias = B2Math.MulSV(joint.LinearSoftness.BiasRate, C);
                    massScale = joint.LinearSoftness.MassScale;
                    impulseScale = joint.LinearSoftness.ImpulseScale;
                }

                Vec2 Cdot = B2Math.Sub(B2Math.Add(vB, B2Math.CrossSV(wB, rB)), B2Math.Add(vA, B2Math.CrossSV(wA, rA)));

                Mat22 K;
                K.Cx.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
                K.Cy.X = -rA.Y * rA.X * iA - rB.Y * rB.X * iB;
                K.Cx.Y = K.Cy.X;
                K.Cy.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
                Vec2 b = B2Math.Solve22(K, B2Math.Add(Cdot, bias));

                Vec2 impulse =
                    (
                        -massScale * b.X - impulseScale * joint.LinearImpulse.X,
                        -massScale * b.Y - impulseScale * joint.LinearImpulse.Y
                    );

                joint.LinearImpulse = B2Math.Add(joint.LinearImpulse, impulse);

                vA = B2Math.MulSub(vA, mA, impulse);
                wA -= iA * B2Math.Cross(rA, impulse);
                vB = B2Math.MulAdd(vB, mB, impulse);
                wB += iB * B2Math.Cross(rB, impulse);
            }

            stateA.LinearVelocity = vA;
            stateA.AngularVelocity = wA;
            stateB.LinearVelocity = vB;
            stateB.AngularVelocity = wB;
        }
    }
}