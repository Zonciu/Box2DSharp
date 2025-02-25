using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public class RevoluteJointFunc
    {
        public static void EnableSpring(JointId jointId, bool enableSpring)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            if (enableSpring != joint.Joint.RevoluteJoint.EnableSpring)
            {
                joint.Joint.RevoluteJoint.EnableSpring = enableSpring;
                joint.Joint.RevoluteJoint.SpringImpulse = 0.0f;
            }
        }

        public static bool IsSpringEnabled(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            return joint.Joint.RevoluteJoint.EnableSpring;
        }

        public static void SetSpringHertz(JointId jointId, float hertz)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            joint.Joint.RevoluteJoint.Hertz = hertz;
        }

        public static float GetSpringHertz(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            return joint.Joint.RevoluteJoint.Hertz;
        }

        public static void SetSpringDampingRatio(JointId jointId, float dampingRatio)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            joint.Joint.RevoluteJoint.DampingRatio = dampingRatio;
        }

        public static float GetSpringDampingRatio(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            return joint.Joint.RevoluteJoint.DampingRatio;
        }

        public static float GetAngle(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            JointSim jointSim = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            Transform transformA = Body.GetBodyTransform(world, jointSim.BodyIdA);
            Transform transformB = Body.GetBodyTransform(world, jointSim.BodyIdB);

            float angle = B2Math.RelativeAngle(transformB.Q, transformA.Q) - jointSim.Joint.RevoluteJoint.ReferenceAngle;
            angle = B2Math.UnwindAngle(angle);
            return angle;
        }

        public static void EnableLimit(JointId jointId, bool enableLimit)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            if (enableLimit != joint.Joint.RevoluteJoint.EnableLimit)
            {
                joint.Joint.RevoluteJoint.EnableLimit = enableLimit;
                joint.Joint.RevoluteJoint.LowerImpulse = 0.0f;
                joint.Joint.RevoluteJoint.UpperImpulse = 0.0f;
            }
        }

        public static bool IsLimitEnabled(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            return joint.Joint.RevoluteJoint.EnableLimit;
        }

        public static float GetLowerLimit(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            return joint.Joint.RevoluteJoint.LowerAngle;
        }

        public static float GetUpperLimit(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            return joint.Joint.RevoluteJoint.UpperAngle;
        }

        public static void SetLimits(JointId jointId, float lower, float upper)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            if (lower != joint.Joint.RevoluteJoint.LowerAngle || upper != joint.Joint.RevoluteJoint.UpperAngle)
            {
                joint.Joint.RevoluteJoint.LowerAngle = Math.Min(lower, upper);
                joint.Joint.RevoluteJoint.UpperAngle = Math.Max(lower, upper);
                joint.Joint.RevoluteJoint.LowerImpulse = 0.0f;
                joint.Joint.RevoluteJoint.UpperImpulse = 0.0f;
            }
        }

        public static void EnableMotor(JointId jointId, bool enableMotor)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            if (enableMotor != joint.Joint.RevoluteJoint.EnableMotor)
            {
                joint.Joint.RevoluteJoint.EnableMotor = enableMotor;
                joint.Joint.RevoluteJoint.MotorImpulse = 0.0f;
            }
        }

        public static bool IsMotorEnabled(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            return joint.Joint.RevoluteJoint.EnableMotor;
        }

        public static void SetMotorSpeed(JointId jointId, float motorSpeed)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            joint.Joint.RevoluteJoint.MotorSpeed = motorSpeed;
        }

        public static float GetMotorSpeed(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            return joint.Joint.RevoluteJoint.MotorSpeed;
        }

        public static float GetMotorTorque(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            return world.InvH * joint.Joint.RevoluteJoint.MotorImpulse;
        }

        public static void SetMaxMotorTorque(JointId jointId, float torque)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            joint.Joint.RevoluteJoint.MaxMotorTorque = torque;
        }

        public static float GetMaxMotorTorque(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.RevoluteJoint);
            return joint.Joint.RevoluteJoint.MaxMotorTorque;
        }

        public static Vec2 GetRevoluteJointForce(World world, JointSim baseSim)
        {
            Vec2 force = B2Math.MulSV(world.InvH, baseSim.Joint.RevoluteJoint.LinearImpulse);
            return force;
        }

        public static float GetRevoluteJointTorque(World world, JointSim baseSim)
        {
            RevoluteJoint revolute = baseSim.Joint.RevoluteJoint;
            float torque = world.InvH * (revolute.MotorImpulse + revolute.LowerImpulse - revolute.UpperImpulse);
            return torque;
        }

        // Point-to-point constraint
        // C = p2 - p1
        // Cdot = v2 - v1
        //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
        // J = [-I -r1_skew I r2_skew ]
        // Identity used:
        // w k % (rx i + ry j) = w * (-ry i + rx j)

        // Motor constraint
        // Cdot = w2 - w1
        // J = [0 0 -1 0 0 1]
        // K = invI1 + invI2

        // Body State
        // The solver operates on the body state. The body state array does not hold static bodies. Static bodies are shared
        // across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
        // workers, even if the values don't change.
        // This causes some trouble when computing anchors. I rotate the anchors using the body rotation every sub-step. For static
        // bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
        // should be minimized.
        //
        // Solution 1:
        // Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be
        // identity. Base separation and angles need to be computed. Manifolds will be behind a frame, but that is probably best if bodies
        // move fast.
        //
        // Solution 2:
        // Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local
        // space. Potentially confusing and bug prone.

        public static void PrepareRevoluteJoint(JointSim baseSim, StepContext context)
        {
            Debug.Assert(baseSim.Type == JointType.RevoluteJoint);

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

            RevoluteJoint joint = baseSim.Joint.RevoluteJoint;

            joint.IndexA = bodyA.SetIndex == SolverSetType.AwakeSet ? localIndexA : Core.NullIndex;
            joint.IndexB = bodyB.SetIndex == SolverSetType.AwakeSet ? localIndexB : Core.NullIndex;

            // initial anchors in world space
            joint.AnchorA = B2Math.RotateVector(bodySimA.Transform.Q, B2Math.Sub(baseSim.LocalOriginAnchorA, bodySimA.LocalCenter));
            joint.AnchorB = B2Math.RotateVector(bodySimB.Transform.Q, B2Math.Sub(baseSim.LocalOriginAnchorB, bodySimB.LocalCenter));
            joint.DeltaCenter = B2Math.Sub(bodySimB.Center, bodySimA.Center);
            joint.DeltaAngle = B2Math.RelativeAngle(bodySimB.Transform.Q, bodySimA.Transform.Q) - joint.ReferenceAngle;
            joint.DeltaAngle = B2Math.UnwindAngle(joint.DeltaAngle);

            float k = iA + iB;
            joint.AxialMass = k > 0.0f ? 1.0f / k : 0.0f;

            joint.SpringSoftness = Softness.MakeSoft(joint.Hertz, joint.DampingRatio, context.H);

            if (context.EnableWarmStarting == false)
            {
                joint.LinearImpulse = Vec2.Zero;
                joint.SpringImpulse = 0.0f;
                joint.MotorImpulse = 0.0f;
                joint.LowerImpulse = 0.0f;
                joint.UpperImpulse = 0.0f;
            }
        }

        public static void WarmStartRevoluteJoint(JointSim baseSim, StepContext context)
        {
            Debug.Assert(baseSim.Type == JointType.RevoluteJoint);

            float mA = baseSim.InvMassA;
            float mB = baseSim.InvMassB;
            float iA = baseSim.InvIA;
            float iB = baseSim.InvIB;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            RevoluteJoint joint = baseSim.Joint.RevoluteJoint;
            ref BodyState stateA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState stateB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 rA = B2Math.RotateVector(stateA.DeltaRotation, joint.AnchorA);
            Vec2 rB = B2Math.RotateVector(stateB.DeltaRotation, joint.AnchorB);

            float axialImpulse = joint.SpringImpulse + joint.MotorImpulse + joint.LowerImpulse - joint.UpperImpulse;

            stateA.LinearVelocity = B2Math.MulSub(stateA.LinearVelocity, mA, joint.LinearImpulse);
            stateA.AngularVelocity -= iA * (B2Math.Cross(rA, joint.LinearImpulse) + axialImpulse);

            stateB.LinearVelocity = B2Math.MulAdd(stateB.LinearVelocity, mB, joint.LinearImpulse);
            stateB.AngularVelocity += iB * (B2Math.Cross(rB, joint.LinearImpulse) + axialImpulse);
        }

        public static void SolveRevoluteJoint(JointSim baseSim, StepContext context, bool useBias)
        {
            Debug.Assert(baseSim.Type == JointType.RevoluteJoint);

            float mA = baseSim.InvMassA;
            float mB = baseSim.InvMassB;
            float iA = baseSim.InvIA;
            float iB = baseSim.InvIB;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            RevoluteJoint joint = baseSim.Joint.RevoluteJoint;

            ref BodyState stateA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState stateB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 vA = stateA.LinearVelocity;
            float wA = stateA.AngularVelocity;
            Vec2 vB = stateB.LinearVelocity;
            float wB = stateB.AngularVelocity;

            bool fixedRotation = (iA + iB == 0.0f);

            // const float maxBias = context.maxBiasVelocity;

            // Solve spring.
            if (joint.EnableSpring && fixedRotation == false)
            {
                float C = B2Math.RelativeAngle(stateB.DeltaRotation, stateA.DeltaRotation) + joint.DeltaAngle;
                float bias = joint.SpringSoftness.BiasRate * C;
                float massScale = joint.SpringSoftness.MassScale;
                float impulseScale = joint.SpringSoftness.ImpulseScale;

                float Cdot = wB - wA;
                float impulse = -massScale * joint.AxialMass * (Cdot + bias) - impulseScale * joint.SpringImpulse;
                joint.SpringImpulse += impulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve motor constraint.
            if (joint.EnableMotor && fixedRotation == false)
            {
                float Cdot = wB - wA - joint.MotorSpeed;
                float impulse = -joint.AxialMass * Cdot;
                float oldImpulse = joint.MotorImpulse;
                float maxImpulse = context.H * joint.MaxMotorTorque;
                joint.MotorImpulse = Math.Clamp(joint.MotorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = joint.MotorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            if (joint.EnableLimit && fixedRotation == false)
            {
                float jointAngle = B2Math.RelativeAngle(stateB.DeltaRotation, stateA.DeltaRotation) + joint.DeltaAngle;
                jointAngle = B2Math.UnwindAngle(jointAngle);

                // Lower limit
                {
                    float C = jointAngle - joint.LowerAngle;
                    float bias = 0.0f;
                    float massScale = 1.0f;
                    float impulseScale = 0.0f;
                    if (C > 0.0f)
                    {
                        // speculation
                        bias = C * context.InvH;
                    }
                    else if (useBias)
                    {
                        bias = context.JointSoftness.BiasRate * C;
                        massScale = context.JointSoftness.MassScale;
                        impulseScale = context.JointSoftness.ImpulseScale;
                    }

                    float Cdot = wB - wA;
                    float impulse = -massScale * joint.AxialMass * (Cdot + bias) - impulseScale * joint.LowerImpulse;
                    float oldImpulse = joint.LowerImpulse;
                    joint.LowerImpulse = Math.Max(joint.LowerImpulse + impulse, 0.0f);
                    impulse = joint.LowerImpulse - oldImpulse;

                    wA -= iA * impulse;
                    wB += iB * impulse;
                }

                // Upper limit
                // Note: signs are flipped to keep C positive when the constraint is satisfied.
                // This also keeps the impulse positive when the limit is active.
                {
                    float C = joint.UpperAngle - jointAngle;
                    float bias = 0.0f;
                    float massScale = 1.0f;
                    float impulseScale = 0.0f;
                    if (C > 0.0f)
                    {
                        // speculation
                        bias = C * context.InvH;
                    }
                    else if (useBias)
                    {
                        bias = context.JointSoftness.BiasRate * C;
                        massScale = context.JointSoftness.MassScale;
                        impulseScale = context.JointSoftness.ImpulseScale;
                    }

                    // sign flipped on Cdot
                    float Cdot = wA - wB;
                    float impulse = -massScale * joint.AxialMass * (Cdot + bias) - impulseScale * joint.LowerImpulse;
                    float oldImpulse = joint.UpperImpulse;
                    joint.UpperImpulse = Math.Max(joint.UpperImpulse + impulse, 0.0f);
                    impulse = joint.UpperImpulse - oldImpulse;

                    // sign flipped on applied impulse
                    wA += iA * impulse;
                    wB -= iB * impulse;
                }
            }

            // Solve point-to-point constraint
            {
                // J = [-I -r1_skew I r2_skew]
                // r_skew = [-ry; rx]
                // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
                //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

                // current anchors
                Vec2 rA = B2Math.RotateVector(stateA.DeltaRotation, joint.AnchorA);
                Vec2 rB = B2Math.RotateVector(stateB.DeltaRotation, joint.AnchorB);

                Vec2 Cdot = B2Math.Sub(B2Math.Add(vB, B2Math.CrossSV(wB, rB)), B2Math.Add(vA, B2Math.CrossSV(wA, rA)));

                Vec2 bias = Vec2.Zero;
                float massScale = 1.0f;
                float impulseScale = 0.0f;
                if (useBias)
                {
                    Vec2 dcA = stateA.DeltaPosition;
                    Vec2 dcB = stateB.DeltaPosition;

                    Vec2 separation = B2Math.Add(B2Math.Add(B2Math.Sub(dcB, dcA), B2Math.Sub(rB, rA)), joint.DeltaCenter);
                    bias = B2Math.MulSV(context.JointSoftness.BiasRate, separation);
                    massScale = context.JointSoftness.MassScale;
                    impulseScale = context.JointSoftness.ImpulseScale;
                }

                Mat22 K;
                K.Cx.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
                K.Cy.X = -rA.Y * rA.X * iA - rB.Y * rB.X * iB;
                K.Cx.Y = K.Cy.X;
                K.Cy.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
                Vec2 b = B2Math.Solve22(K, B2Math.Add(Cdot, bias));

                Vec2 impulse;
                impulse.X = -massScale * b.X - impulseScale * joint.LinearImpulse.X;
                impulse.Y = -massScale * b.Y - impulseScale * joint.LinearImpulse.Y;
                joint.LinearImpulse.X += impulse.X;
                joint.LinearImpulse.Y += impulse.Y;

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

        public static void DrawRevoluteJoint(DebugDrawBase draw, JointSim baseSim, Transform transformA, Transform transformB, float drawSize)
        {
            Debug.Assert(baseSim.Type == JointType.RevoluteJoint);

            RevoluteJoint joint = baseSim.Joint.RevoluteJoint;

            Vec2 pA = B2Math.TransformPoint(transformA, baseSim.LocalOriginAnchorA);
            Vec2 pB = B2Math.TransformPoint(transformB, baseSim.LocalOriginAnchorB);

            B2HexColor c1 = B2HexColor.Gray7;
            B2HexColor c2 = B2HexColor.Green;
            B2HexColor c3 = B2HexColor.Red;

            float L = drawSize;

            // draw.DrawPoint(pA, 3.0f, b2_colorGray40, draw.context);
            // draw.DrawPoint(pB, 3.0f, b2_colorLightBlue, draw.context);
            draw.DrawCircle(pB, L, c1, draw.Context);

            float angle = B2Math.RelativeAngle(transformB.Q, transformA.Q);

            Rot rot = B2Math.MakeRot(angle);
            Vec2 r = (L * rot.C, L * rot.S);
            Vec2 pC = B2Math.Add(pB, r);
            draw.DrawSegment(pB, pC, c1, draw.Context);

            if (draw.DrawJointExtras)
            {
                float jointAngle = B2Math.UnwindAngle(angle - joint.ReferenceAngle);
                var deg = 180.0f * jointAngle / B2Math.Pi;
                draw.DrawString(pC, $" {deg:F1} deg", draw.Context);
            }

            float lowerAngle = joint.LowerAngle + joint.ReferenceAngle;
            float upperAngle = joint.UpperAngle + joint.ReferenceAngle;

            if (joint.EnableLimit)
            {
                Rot rotLo = B2Math.MakeRot(lowerAngle);
                Vec2 rlo = (L * rotLo.C, L * rotLo.S);

                Rot rotHi = B2Math.MakeRot(upperAngle);
                Vec2 rhi = (L * rotHi.C, L * rotHi.S);

                draw.DrawSegment(pB, B2Math.Add(pB, rlo), c2, draw.Context);
                draw.DrawSegment(pB, B2Math.Add(pB, rhi), c3, draw.Context);

                Rot rotRef = B2Math.MakeRot(joint.ReferenceAngle);
                Vec2 refV = (L * rotRef.C, L * rotRef.S);
                draw.DrawSegment(pB, B2Math.Add(pB, refV), B2HexColor.Blue, draw.Context);
            }

            B2HexColor color = B2HexColor.Gold;
            draw.DrawSegment(transformA.P, pA, color, draw.Context);
            draw.DrawSegment(pA, pB, color, draw.Context);
            draw.DrawSegment(transformB.P, pB, color, draw.Context);
        }
    }
}