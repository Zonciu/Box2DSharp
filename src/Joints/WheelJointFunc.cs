using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public class WheelJointFunc
    {
        public static void EnableSpring(JointId jointId, bool enableSpring)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);

            if (enableSpring != joint.Joint.WheelJoint.EnableSpring)
            {
                joint.Joint.WheelJoint.EnableSpring = enableSpring;
                joint.Joint.WheelJoint.SpringImpulse = 0.0f;
            }
        }

        public static bool IsSpringEnabled(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            return joint.Joint.WheelJoint.EnableSpring;
        }

        public static void SetSpringHertz(JointId jointId, float hertz)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            joint.Joint.WheelJoint.Hertz = hertz;
        }

        public static float GetSpringHertz(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            return joint.Joint.WheelJoint.Hertz;
        }

        public static void SetSpringDampingRatio(JointId jointId, float dampingRatio)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            joint.Joint.WheelJoint.DampingRatio = dampingRatio;
        }

        public static float GetSpringDampingRatio(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            return joint.Joint.WheelJoint.DampingRatio;
        }

        public static void EnableLimit(JointId jointId, bool enableLimit)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            if (joint.Joint.WheelJoint.EnableLimit != enableLimit)
            {
                joint.Joint.WheelJoint.LowerImpulse = 0.0f;
                joint.Joint.WheelJoint.UpperImpulse = 0.0f;
                joint.Joint.WheelJoint.EnableLimit = enableLimit;
            }
        }

        public static bool IsLimitEnabled(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            return joint.Joint.WheelJoint.EnableLimit;
        }

        public static float GetLowerLimit(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            return joint.Joint.WheelJoint.LowerTranslation;
        }

        public static float GetUpperLimit(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            return joint.Joint.WheelJoint.UpperTranslation;
        }

        public static void SetLimits(JointId jointId, float lower, float upper)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            if (lower != joint.Joint.WheelJoint.LowerTranslation || upper != joint.Joint.WheelJoint.UpperTranslation)
            {
                joint.Joint.WheelJoint.LowerTranslation = Math.Min(lower, upper);
                joint.Joint.WheelJoint.UpperTranslation = Math.Max(lower, upper);
                joint.Joint.WheelJoint.LowerImpulse = 0.0f;
                joint.Joint.WheelJoint.UpperImpulse = 0.0f;
            }
        }

        public static void EnableMotor(JointId jointId, bool enableMotor)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            if (joint.Joint.WheelJoint.EnableMotor != enableMotor)
            {
                joint.Joint.WheelJoint.MotorImpulse = 0.0f;
                joint.Joint.WheelJoint.EnableMotor = enableMotor;
            }
        }

        public static bool IsMotorEnabled(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            return joint.Joint.WheelJoint.EnableMotor;
        }

        public static void SetMotorSpeed(JointId jointId, float motorSpeed)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            joint.Joint.WheelJoint.MotorSpeed = motorSpeed;
        }

        public static float GetMotorSpeed(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            return joint.Joint.WheelJoint.MotorSpeed;
        }

        public static float GetMotorTorque(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            return world.InvH * joint.Joint.WheelJoint.MotorImpulse;
        }

        public static void SetMaxMotorTorque(JointId jointId, float torque)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            joint.Joint.WheelJoint.MaxMotorTorque = torque;
        }

        public static float GetMaxMotorTorque(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.WheelJoint);
            return joint.Joint.WheelJoint.MaxMotorTorque;
        }

        public static Vec2 GetWheelJointForce(World world, JointSim baseSim)
        {
            WheelJoint joint = baseSim.Joint.WheelJoint;

            // This is a frame behind
            Vec2 axisA = joint.AxisA;
            Vec2 perpA = B2Math.LeftPerp(axisA);

            float perpForce = world.InvH * joint.PerpImpulse;
            float axialForce = world.InvH * (joint.SpringImpulse + joint.LowerImpulse - joint.UpperImpulse);

            Vec2 force = B2Math.Add(B2Math.MulSV(perpForce, perpA), B2Math.MulSV(axialForce, axisA));
            return force;
        }

        public static float GetWheelJointTorque(World world, JointSim baseSim)
        {
            return world.InvH * baseSim.Joint.WheelJoint.MotorImpulse;
        }

        // Linear constraint (point-to-line)
        // d = pB - pA = xB + rB - xA - rA
        // C = dot(ay, d)
        // Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
        //      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
        // J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

        // Spring linear constraint
        // C = dot(ax, d)
        // Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
        // J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

        // Motor rotational constraint
        // Cdot = wB - wA
        // J = [0 0 -1 0 0 1]

        public static void PrepareWheelJoint(JointSim baseSim, StepContext context)
        {
            Debug.Assert(baseSim.Type == JointType.WheelJoint);

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

            WheelJoint joint = baseSim.Joint.WheelJoint;

            joint.IndexA = bodyA.SetIndex == SolverSetType.AwakeSet ? localIndexA : Core.NullIndex;
            joint.IndexB = bodyB.SetIndex == SolverSetType.AwakeSet ? localIndexB : Core.NullIndex;

            Rot qA = bodySimA.Transform.Q;
            Rot qB = bodySimB.Transform.Q;

            joint.AnchorA = B2Math.RotateVector(qA, B2Math.Sub(baseSim.LocalOriginAnchorA, bodySimA.LocalCenter));
            joint.AnchorB = B2Math.RotateVector(qB, B2Math.Sub(baseSim.LocalOriginAnchorB, bodySimB.LocalCenter));
            joint.AxisA = B2Math.RotateVector(qA, joint.LocalAxisA);
            joint.DeltaCenter = B2Math.Sub(bodySimB.Center, bodySimA.Center);

            Vec2 rA = joint.AnchorA;
            Vec2 rB = joint.AnchorB;

            Vec2 d = B2Math.Add(joint.DeltaCenter, B2Math.Sub(rB, rA));
            Vec2 axisA = joint.AxisA;
            Vec2 perpA = B2Math.LeftPerp(axisA);

            // perpendicular constraint (keep wheel on line)
            float s1 = B2Math.Cross(B2Math.Add(d, rA), perpA);
            float s2 = B2Math.Cross(rB, perpA);

            float kp = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            joint.PerpMass = kp > 0.0f ? 1.0f / kp : 0.0f;

            // spring constraint
            float a1 = B2Math.Cross(B2Math.Add(d, rA), axisA);
            float a2 = B2Math.Cross(rB, axisA);

            float ka = mA + mB + iA * a1 * a1 + iB * a2 * a2;
            joint.AxialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

            joint.SpringSoftness = Softness.MakeSoft(joint.Hertz, joint.DampingRatio, context.H);

            float km = iA + iB;
            joint.MotorMass = km > 0.0f ? 1.0f / km : 0.0f;

            if (context.EnableWarmStarting == false)
            {
                joint.PerpImpulse = 0.0f;
                joint.SpringImpulse = 0.0f;
                joint.MotorImpulse = 0.0f;
                joint.LowerImpulse = 0.0f;
                joint.UpperImpulse = 0.0f;
            }
        }

        public static void WarmStartWheelJoint(JointSim baseSim, StepContext context)
        {
            Debug.Assert(baseSim.Type == JointType.WheelJoint);

            float mA = baseSim.InvMassA;
            float mB = baseSim.InvMassB;
            float iA = baseSim.InvIA;
            float iB = baseSim.InvIB;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            WheelJoint joint = baseSim.Joint.WheelJoint;

            ref BodyState stateA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState stateB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 rA = B2Math.RotateVector(stateA.DeltaRotation, joint.AnchorA);
            Vec2 rB = B2Math.RotateVector(stateB.DeltaRotation, joint.AnchorB);

            Vec2 d = B2Math.Add(B2Math.Add(B2Math.Sub(stateB.DeltaPosition, stateA.DeltaPosition), joint.DeltaCenter), B2Math.Sub(rB, rA));
            Vec2 axisA = B2Math.RotateVector(stateA.DeltaRotation, joint.AxisA);
            Vec2 perpA = B2Math.LeftPerp(axisA);

            float a1 = B2Math.Cross(B2Math.Add(d, rA), axisA);
            float a2 = B2Math.Cross(rB, axisA);
            float s1 = B2Math.Cross(B2Math.Add(d, rA), perpA);
            float s2 = B2Math.Cross(rB, perpA);

            float axialImpulse = joint.SpringImpulse + joint.LowerImpulse - joint.UpperImpulse;

            Vec2 P = B2Math.Add(B2Math.MulSV(axialImpulse, axisA), B2Math.MulSV(joint.PerpImpulse, perpA));
            float LA = axialImpulse * a1 + joint.PerpImpulse * s1 + joint.MotorImpulse;
            float LB = axialImpulse * a2 + joint.PerpImpulse * s2 + joint.MotorImpulse;

            stateA.LinearVelocity = B2Math.MulSub(stateA.LinearVelocity, mA, P);
            stateA.AngularVelocity -= iA * LA;
            stateB.LinearVelocity = B2Math.MulAdd(stateB.LinearVelocity, mB, P);
            stateB.AngularVelocity += iB * LB;
        }

        public static void SolveWheelJoint(JointSim baseSim, StepContext context, bool useBias)
        {
            Debug.Assert(baseSim.Type == JointType.WheelJoint);

            float mA = baseSim.InvMassA;
            float mB = baseSim.InvMassB;
            float iA = baseSim.InvIA;
            float iB = baseSim.InvIB;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            WheelJoint joint = baseSim.Joint.WheelJoint;

            // This is a dummy body to represent a static body since static bodies don't have a solver body.
            BodyState dummyBody = new();

            ref BodyState stateA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState stateB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 vA = stateA.LinearVelocity;
            float wA = stateA.AngularVelocity;
            Vec2 vB = stateB.LinearVelocity;
            float wB = stateB.AngularVelocity;

            bool fixedRotation = (iA + iB == 0.0f);

            // current anchors
            Vec2 rA = B2Math.RotateVector(stateA.DeltaRotation, joint.AnchorA);
            Vec2 rB = B2Math.RotateVector(stateB.DeltaRotation, joint.AnchorB);

            Vec2 d = B2Math.Add(B2Math.Add(B2Math.Sub(stateB.DeltaPosition, stateA.DeltaPosition), joint.DeltaCenter), B2Math.Sub(rB, rA));
            Vec2 axisA = B2Math.RotateVector(stateA.DeltaRotation, joint.AxisA);
            float translation = B2Math.Dot(axisA, d);

            float a1 = B2Math.Cross(B2Math.Add(d, rA), axisA);
            float a2 = B2Math.Cross(rB, axisA);

            // motor constraint
            if (joint.EnableMotor && fixedRotation == false)
            {
                float Cdot = wB - wA - joint.MotorSpeed;
                float impulse = -joint.MotorMass * Cdot;
                float oldImpulse = joint.MotorImpulse;
                float maxImpulse = context.H * joint.MaxMotorTorque;
                joint.MotorImpulse = Math.Clamp(joint.MotorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = joint.MotorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // spring constraint
            if (joint.EnableSpring)
            {
                // This is a real spring and should be applied even during relax
                float C = translation;
                float bias = joint.SpringSoftness.BiasRate * C;
                float massScale = joint.SpringSoftness.MassScale;
                float impulseScale = joint.SpringSoftness.ImpulseScale;

                float Cdot = B2Math.Dot(axisA, B2Math.Sub(vB, vA)) + a2 * wB - a1 * wA;
                float impulse = -massScale * joint.AxialMass * (Cdot + bias) - impulseScale * joint.SpringImpulse;
                joint.SpringImpulse += impulse;

                Vec2 P = B2Math.MulSV(impulse, axisA);
                float LA = impulse * a1;
                float LB = impulse * a2;

                vA = B2Math.MulSub(vA, mA, P);
                wA -= iA * LA;
                vB = B2Math.MulAdd(vB, mB, P);
                wB += iB * LB;
            }

            if (joint.EnableLimit)
            {
                translation = B2Math.Dot(axisA, d);

                // Lower limit
                {
                    float C = translation - joint.LowerTranslation;
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

                    float Cdot = B2Math.Dot(axisA, B2Math.Sub(vB, vA)) + a2 * wB - a1 * wA;
                    float impulse = -massScale * joint.AxialMass * (Cdot + bias) - impulseScale * joint.LowerImpulse;
                    float oldImpulse = joint.LowerImpulse;
                    joint.LowerImpulse = Math.Max(oldImpulse + impulse, 0.0f);
                    impulse = joint.LowerImpulse - oldImpulse;

                    Vec2 P = B2Math.MulSV(impulse, axisA);
                    float LA = impulse * a1;
                    float LB = impulse * a2;

                    vA = B2Math.MulSub(vA, mA, P);
                    wA -= iA * LA;
                    vB = B2Math.MulAdd(vB, mB, P);
                    wB += iB * LB;
                }

                // Upper limit
                // Note: signs are flipped to keep C positive when the constraint is satisfied.
                // This also keeps the impulse positive when the limit is active.
                {
                    // sign flipped
                    float C = joint.UpperTranslation - translation;
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
                    float Cdot = B2Math.Dot(axisA, B2Math.Sub(vA, vB)) + a1 * wA - a2 * wB;
                    float impulse = -massScale * joint.AxialMass * (Cdot + bias) - impulseScale * joint.UpperImpulse;
                    float oldImpulse = joint.UpperImpulse;
                    joint.UpperImpulse = Math.Max(oldImpulse + impulse, 0.0f);
                    impulse = joint.UpperImpulse - oldImpulse;

                    Vec2 P = B2Math.MulSV(impulse, axisA);
                    float LA = impulse * a1;
                    float LB = impulse * a2;

                    // sign flipped on applied impulse
                    vA = B2Math.MulAdd(vA, mA, P);
                    wA += iA * LA;
                    vB = B2Math.MulSub(vB, mB, P);
                    wB -= iB * LB;
                }
            }

            // point to line constraint
            {
                Vec2 perpA = B2Math.LeftPerp(axisA);

                float bias = 0.0f;
                float massScale = 1.0f;
                float impulseScale = 0.0f;
                if (useBias)
                {
                    float C = B2Math.Dot(perpA, d);
                    bias = context.JointSoftness.BiasRate * C;
                    massScale = context.JointSoftness.MassScale;
                    impulseScale = context.JointSoftness.ImpulseScale;
                }

                float s1 = B2Math.Cross(B2Math.Add(d, rA), perpA);
                float s2 = B2Math.Cross(rB, perpA);
                float Cdot = B2Math.Dot(perpA, B2Math.Sub(vB, vA)) + s2 * wB - s1 * wA;

                float impulse = -massScale * joint.PerpMass * (Cdot + bias) - impulseScale * joint.PerpImpulse;
                joint.PerpImpulse += impulse;

                Vec2 P = B2Math.MulSV(impulse, perpA);
                float LA = impulse * s1;
                float LB = impulse * s2;

                vA = B2Math.MulSub(vA, mA, P);
                wA -= iA * LA;
                vB = B2Math.MulAdd(vB, mB, P);
                wB += iB * LB;
            }

            stateA.LinearVelocity = vA;
            stateA.AngularVelocity = wA;
            stateB.LinearVelocity = vB;
            stateB.AngularVelocity = wB;
        }

        public static void DrawWheelJoint(DebugDrawBase draw, JointSim baseSim, Transform transformA, Transform transformB)
        {
            Debug.Assert(baseSim.Type == JointType.WheelJoint);

            WheelJoint joint = baseSim.Joint.WheelJoint;

            Vec2 pA = B2Math.TransformPoint(transformA, baseSim.LocalOriginAnchorA);
            Vec2 pB = B2Math.TransformPoint(transformB, baseSim.LocalOriginAnchorB);
            Vec2 axis = B2Math.RotateVector(transformA.Q, joint.LocalAxisA);

            B2HexColor c1 = B2HexColor.Gray7;
            B2HexColor c2 = B2HexColor.Green;
            B2HexColor c3 = B2HexColor.Red;
            B2HexColor c4 = B2HexColor.Gray4;
            B2HexColor c5 = B2HexColor.Blue;

            draw.DrawSegment(pA, pB, c5, draw.Context);

            if (joint.EnableLimit)
            {
                Vec2 lower = B2Math.MulAdd(pA, joint.LowerTranslation, axis);
                Vec2 upper = B2Math.MulAdd(pA, joint.UpperTranslation, axis);
                Vec2 perp = B2Math.LeftPerp(axis);
                draw.DrawSegment(lower, upper, c1, draw.Context);
                draw.DrawSegment(B2Math.MulSub(lower, 0.1f, perp), B2Math.MulAdd(lower, 0.1f, perp), c2, draw.Context);
                draw.DrawSegment(B2Math.MulSub(upper, 0.1f, perp), B2Math.MulAdd(upper, 0.1f, perp), c3, draw.Context);
            }
            else
            {
                draw.DrawSegment(B2Math.MulSub(pA, 1.0f, axis), B2Math.MulAdd(pA, 1.0f, axis), c1, draw.Context);
            }

            draw.DrawPoint(pA, 5.0f, c1, draw.Context);
            draw.DrawPoint(pB, 5.0f, c4, draw.Context);
        }
    }
}