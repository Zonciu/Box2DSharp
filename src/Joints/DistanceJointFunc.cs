using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public static class DistanceJointFunc
    {
        public static void SetLength(JointId jointId, float length)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            DistanceJoint joint = baseSim.Joint.DistanceJoint;

            joint.Length = Math.Clamp(length, Core.LinearSlop, Core.Huge);
            joint.Impulse = 0.0f;
            joint.LowerImpulse = 0.0f;
            joint.UpperImpulse = 0.0f;
        }

        public static float GetLength(JointId jointId)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            DistanceJoint joint = baseSim.Joint.DistanceJoint;
            return joint.Length;
        }

        public static void EnableLimit(JointId jointId, bool enableLimit)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            DistanceJoint joint = baseSim.Joint.DistanceJoint;
            joint.EnableLimit = enableLimit;
        }

        public static bool IsLimitEnabled(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            return joint.Joint.DistanceJoint.EnableLimit;
        }

        public static void SetLengthRange(JointId jointId, float minLength, float maxLength)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            DistanceJoint joint = baseSim.Joint.DistanceJoint;

            minLength = Math.Clamp(minLength, Core.LinearSlop, Core.Huge);
            maxLength = Math.Clamp(maxLength, Core.LinearSlop, Core.Huge);
            joint.MinLength = Math.Min(minLength, maxLength);
            joint.MaxLength = Math.Max(minLength, maxLength);
            joint.Impulse = 0.0f;
            joint.LowerImpulse = 0.0f;
            joint.UpperImpulse = 0.0f;
        }

        public static float GetMinLength(JointId jointId)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            DistanceJoint joint = baseSim.Joint.DistanceJoint;
            return joint.MinLength;
        }

        public static float GetMaxLength(JointId jointId)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            DistanceJoint joint = baseSim.Joint.DistanceJoint;
            return joint.MaxLength;
        }

        public static float GetCurrentLength(JointId jointId)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);

            World world = World.GetWorld(jointId.World0);
            Debug.Assert(world.Locked == false);
            if (world.Locked)
            {
                return 0.0f;
            }

            Transform transformA = Body.GetBodyTransform(world, baseSim.BodyIdA);
            Transform transformB = Body.GetBodyTransform(world, baseSim.BodyIdB);

            Vec2 pA = B2Math.TransformPoint(transformA, baseSim.LocalOriginAnchorA);
            Vec2 pB = B2Math.TransformPoint(transformB, baseSim.LocalOriginAnchorB);
            Vec2 d = B2Math.Sub(pB, pA);
            float length = d.Length;
            return length;
        }

        public static void EnableSpring(JointId jointId, bool enableSpring)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            baseSim.Joint.DistanceJoint.EnableSpring = enableSpring;
        }

        public static bool IsSpringEnabled(JointId jointId)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            return baseSim.Joint.DistanceJoint.EnableSpring;
        }

        public static void SetSpringHertz(JointId jointId, float hertz)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            baseSim.Joint.DistanceJoint.Hertz = hertz;
        }

        public static void SetSpringDampingRatio(JointId jointId, float dampingRatio)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            baseSim.Joint.DistanceJoint.DampingRatio = dampingRatio;
        }

        public static float GetSpringHertz(JointId jointId)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            DistanceJoint joint = baseSim.Joint.DistanceJoint;
            return joint.Hertz;
        }

        public static float GetSpringDampingRatio(JointId jointId)
        {
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            DistanceJoint joint = baseSim.Joint.DistanceJoint;
            return joint.DampingRatio;
        }

        public static void EnableMotor(JointId jointId, bool enableMotor)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            if (enableMotor != joint.Joint.DistanceJoint.EnableMotor)
            {
                joint.Joint.DistanceJoint.EnableMotor = enableMotor;
                joint.Joint.DistanceJoint.MotorImpulse = 0.0f;
            }
        }

        public static bool IsMotorEnabled(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            return joint.Joint.DistanceJoint.EnableMotor;
        }

        public static void SetMotorSpeed(JointId jointId, float motorSpeed)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            joint.Joint.DistanceJoint.MotorSpeed = motorSpeed;
        }

        public static float GetMotorSpeed(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            return joint.Joint.DistanceJoint.MotorSpeed;
        }

        public static float GetMotorForce(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            JointSim baseSim = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            return world.InvH * baseSim.Joint.DistanceJoint.MotorImpulse;
        }

        public static void SetMaxMotorForce(JointId jointId, float force)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            joint.Joint.DistanceJoint.MaxMotorForce = force;
        }

        public static float GetMaxMotorForce(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.DistanceJoint);
            return joint.Joint.DistanceJoint.MaxMotorForce;
        }

        public static Vec2 GetDistanceJointForce(World world, JointSim baseSim)
        {
            DistanceJoint joint = baseSim.Joint.DistanceJoint;

            Transform transformA = Body.GetBodyTransform(world, baseSim.BodyIdA);
            Transform transformB = Body.GetBodyTransform(world, baseSim.BodyIdB);

            Vec2 pA = B2Math.TransformPoint(transformA, baseSim.LocalOriginAnchorA);
            Vec2 pB = B2Math.TransformPoint(transformB, baseSim.LocalOriginAnchorB);
            Vec2 d = B2Math.Sub(pB, pA);
            Vec2 axis = d.Normalize;
            float force = (joint.Impulse + joint.LowerImpulse - joint.UpperImpulse + joint.MotorImpulse) * world.InvH;
            return B2Math.MulSV(force, axis);
        }

        // 1-D constrained system
        // m (v2 - v1) = lambda
        // v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
        // x2 = x1 + h * v2

        // 1-D mass-damper-spring system
        // m (v2 - v1) + h * d * v2 + h * k *

        // C = norm(p2 - p1) - L
        // u = (p2 - p1) / norm(p2 - p1)
        // Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
        // J = [-u -cross(r1, u) u cross(r2, u)]
        // K = J * invM * JT
        //   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2
        public static void PrepareDistanceJoint(JointSim baseSim, StepContext context)
        {
            Debug.Assert(baseSim.Type == JointType.DistanceJoint);

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

            Debug.Assert(0 <= bodyA.LocalIndex && bodyA.LocalIndex <= setA.Sims.Count);
            Debug.Assert(0 <= bodyB.LocalIndex && bodyB.LocalIndex <= setB.Sims.Count);

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

            DistanceJoint joint = baseSim.Joint.DistanceJoint;

            joint.IndexA = bodyA.SetIndex == SolverSetType.AwakeSet ? bodyA.LocalIndex : Core.NullIndex;
            joint.IndexB = bodyB.SetIndex == SolverSetType.AwakeSet ? bodyB.LocalIndex : Core.NullIndex;

            // initial anchors in world space 在世界坐标系的锚点
            joint.AnchorA = B2Math.RotateVector(bodySimA.Transform.Q, B2Math.Sub(baseSim.LocalOriginAnchorA, bodySimA.LocalCenter));
            joint.AnchorB = B2Math.RotateVector(bodySimB.Transform.Q, B2Math.Sub(baseSim.LocalOriginAnchorB, bodySimB.LocalCenter));
            joint.DeltaCenter = B2Math.Sub(bodySimB.Center, bodySimA.Center);

            Vec2 rA = joint.AnchorA;
            Vec2 rB = joint.AnchorB;
            Vec2 separation = B2Math.Add(B2Math.Sub(rB, rA), joint.DeltaCenter);
            Vec2 axis = separation.Normalize;

            // compute effective mass
            float crA = B2Math.Cross(rA, axis);
            float crB = B2Math.Cross(rB, axis);
            float k = mA + mB + iA * crA * crA + iB * crB * crB;
            joint.AxialMass = k > 0.0f ? 1.0f / k : 0.0f;

            joint.DistanceSoftness = Softness.MakeSoft(joint.Hertz, joint.DampingRatio, context.H);

            if (context.EnableWarmStarting == false)
            {
                joint.Impulse = 0.0f;
                joint.LowerImpulse = 0.0f;
                joint.UpperImpulse = 0.0f;
                joint.MotorImpulse = 0.0f;
            }
        }

        public static void WarmStartDistanceJoint(JointSim baseSim, StepContext context)
        {
            Debug.Assert(baseSim.Type == JointType.DistanceJoint);

            float mA = baseSim.InvMassA;
            float mB = baseSim.InvMassB;
            float iA = baseSim.InvIA;
            float iB = baseSim.InvIB;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            DistanceJoint joint = baseSim.Joint.DistanceJoint;
            ref BodyState stateA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState stateB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 rA = B2Math.RotateVector(stateA.DeltaRotation, joint.AnchorA);
            Vec2 rB = B2Math.RotateVector(stateB.DeltaRotation, joint.AnchorB);

            Vec2 ds = B2Math.Add(B2Math.Sub(stateB.DeltaPosition, stateA.DeltaPosition), B2Math.Sub(rB, rA));
            Vec2 separation = B2Math.Add(joint.DeltaCenter, ds);
            Vec2 axis = separation.Normalize;

            float axialImpulse = joint.Impulse + joint.LowerImpulse - joint.UpperImpulse + joint.MotorImpulse;
            Vec2 P = B2Math.MulSV(axialImpulse, axis);

            stateA.LinearVelocity = B2Math.MulSub(stateA.LinearVelocity, mA, P);
            stateA.AngularVelocity -= iA * B2Math.Cross(rA, P);
            stateB.LinearVelocity = B2Math.MulAdd(stateB.LinearVelocity, mB, P);
            stateB.AngularVelocity += iB * B2Math.Cross(rB, P);
        }

        public static void SolveDistanceJoint(JointSim baseSim, StepContext context, bool useBias)
        {
            Debug.Assert(baseSim.Type == JointType.DistanceJoint);

            float mA = baseSim.InvMassA;
            float mB = baseSim.InvMassB;
            float iA = baseSim.InvIA;
            float iB = baseSim.InvIB;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            DistanceJoint joint = baseSim.Joint.DistanceJoint;
            ref BodyState stateA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState stateB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 vA = stateA.LinearVelocity;
            float wA = stateA.AngularVelocity;
            Vec2 vB = stateB.LinearVelocity;
            float wB = stateB.AngularVelocity;

            // current anchors
            Vec2 rA = B2Math.RotateVector(stateA.DeltaRotation, joint.AnchorA);
            Vec2 rB = B2Math.RotateVector(stateB.DeltaRotation, joint.AnchorB);

            // current separation
            Vec2 ds = B2Math.Add(B2Math.Sub(stateB.DeltaPosition, stateA.DeltaPosition), B2Math.Sub(rB, rA));
            Vec2 separation = B2Math.Add(joint.DeltaCenter, ds);

            (Vec2 axis, float length) = separation.GetLengthAndNormalize();

            // joint is soft if
            // - spring is enabled
            // - and (joint limit is disabled or limits are not equal)
            if (joint.EnableSpring && (joint.MinLength < joint.MaxLength || joint.EnableLimit == false))
            {
                // spring
                if (joint.Hertz > 0.0f)
                {
                    // Cdot = dot(u, v + cross(w, r))
                    Vec2 vr = B2Math.Add(B2Math.Sub(vB, vA), B2Math.Sub(B2Math.CrossSV(wB, rB), B2Math.CrossSV(wA, rA)));
                    float Cdot = B2Math.Dot(axis, vr);
                    float C = length - joint.Length;
                    float bias = joint.DistanceSoftness.BiasRate * C;

                    float m = joint.DistanceSoftness.MassScale * joint.AxialMass;
                    float impulse = -m * (Cdot + bias) - joint.DistanceSoftness.ImpulseScale * joint.Impulse;
                    joint.Impulse += impulse;

                    Vec2 P = B2Math.MulSV(impulse, axis);
                    vA = B2Math.MulSub(vA, mA, P);
                    wA -= iA * B2Math.Cross(rA, P);
                    vB = B2Math.MulAdd(vB, mB, P);
                    wB += iB * B2Math.Cross(rB, P);
                }

                if (joint.EnableLimit)
                {
                    // lower limit
                    {
                        Vec2 vr = B2Math.Add(B2Math.Sub(vB, vA), B2Math.Sub(B2Math.CrossSV(wB, rB), B2Math.CrossSV(wA, rA)));
                        float Cdot = B2Math.Dot(axis, vr);

                        float C = length - joint.MinLength;

                        float bias = 0.0f;
                        float massCoeff = 1.0f;
                        float impulseCoeff = 0.0f;
                        if (C > 0.0f)
                        {
                            // speculative
                            bias = C * context.InvH;
                        }
                        else if (useBias)
                        {
                            bias = context.JointSoftness.BiasRate * C;
                            massCoeff = context.JointSoftness.MassScale;
                            impulseCoeff = context.JointSoftness.ImpulseScale;
                        }

                        float impulse = -massCoeff * joint.AxialMass * (Cdot + bias) - impulseCoeff * joint.LowerImpulse;
                        float newImpulse = Math.Max(0.0f, joint.LowerImpulse + impulse);
                        impulse = newImpulse - joint.LowerImpulse;
                        joint.LowerImpulse = newImpulse;

                        Vec2 P = B2Math.MulSV(impulse, axis);
                        vA = B2Math.MulSub(vA, mA, P);
                        wA -= iA * B2Math.Cross(rA, P);
                        vB = B2Math.MulAdd(vB, mB, P);
                        wB += iB * B2Math.Cross(rB, P);
                    }

                    // upper
                    {
                        Vec2 vr = B2Math.Add(B2Math.Sub(vA, vB), B2Math.Sub(B2Math.CrossSV(wA, rA), B2Math.CrossSV(wB, rB)));
                        float Cdot = B2Math.Dot(axis, vr);

                        float C = joint.MaxLength - length;

                        float bias = 0.0f;
                        float massScale = 1.0f;
                        float impulseScale = 0.0f;
                        if (C > 0.0f)
                        {
                            // speculative
                            bias = C * context.InvH;
                        }
                        else if (useBias)
                        {
                            bias = context.JointSoftness.BiasRate * C;
                            massScale = context.JointSoftness.MassScale;
                            impulseScale = context.JointSoftness.ImpulseScale;
                        }

                        float impulse = -massScale * joint.AxialMass * (Cdot + bias) - impulseScale * joint.UpperImpulse;
                        float newImpulse = Math.Max(0.0f, joint.UpperImpulse + impulse);
                        impulse = newImpulse - joint.UpperImpulse;
                        joint.UpperImpulse = newImpulse;

                        Vec2 P = B2Math.MulSV(-impulse, axis);
                        vA = B2Math.MulSub(vA, mA, P);
                        wA -= iA * B2Math.Cross(rA, P);
                        vB = B2Math.MulAdd(vB, mB, P);
                        wB += iB * B2Math.Cross(rB, P);
                    }
                }

                if (joint.EnableMotor)
                {
                    Vec2 vr = B2Math.Add(B2Math.Sub(vB, vA), B2Math.Sub(B2Math.CrossSV(wB, rB), B2Math.CrossSV(wA, rA)));
                    float Cdot = B2Math.Dot(axis, vr);
                    float impulse = joint.AxialMass * (joint.MotorSpeed - Cdot);
                    float oldImpulse = joint.MotorImpulse;
                    float maxImpulse = context.H * joint.MaxMotorForce;
                    joint.MotorImpulse = Math.Clamp(joint.MotorImpulse + impulse, -maxImpulse, maxImpulse);
                    impulse = joint.MotorImpulse - oldImpulse;

                    Vec2 P = B2Math.MulSV(impulse, axis);
                    vA = B2Math.MulSub(vA, mA, P);
                    wA -= iA * B2Math.Cross(rA, P);
                    vB = B2Math.MulAdd(vB, mB, P);
                    wB += iB * B2Math.Cross(rB, P);
                }
            }
            else
            {
                // rigid constraint
                Vec2 vr = B2Math.Add(B2Math.Sub(vB, vA), B2Math.Sub(B2Math.CrossSV(wB, rB), B2Math.CrossSV(wA, rA)));
                float Cdot = B2Math.Dot(axis, vr);

                float C = length - joint.Length;

                float bias = 0.0f;
                float massScale = 1.0f;
                float impulseScale = 0.0f;
                if (useBias)
                {
                    bias = context.JointSoftness.BiasRate * C;
                    massScale = context.JointSoftness.MassScale;
                    impulseScale = context.JointSoftness.ImpulseScale;
                }

                float impulse = -massScale * joint.AxialMass * (Cdot + bias) - impulseScale * joint.Impulse;
                joint.Impulse += impulse;

                Vec2 P = B2Math.MulSV(impulse, axis);
                vA = B2Math.MulSub(vA, mA, P);
                wA -= iA * B2Math.Cross(rA, P);
                vB = B2Math.MulAdd(vB, mB, P);
                wB += iB * B2Math.Cross(rB, P);
            }

            stateA.LinearVelocity = vA;
            stateA.AngularVelocity = wA;
            stateB.LinearVelocity = vB;
            stateB.AngularVelocity = wB;
        }

        public static void DrawDistanceJoint(DebugDrawBase draw, JointSim baseSim, Transform transformA, Transform transformB)
        {
            Debug.Assert(baseSim.Type == JointType.DistanceJoint);

            DistanceJoint joint = baseSim.Joint.DistanceJoint;

            Vec2 pA = B2Math.TransformPoint(transformA, baseSim.LocalOriginAnchorA);
            Vec2 pB = B2Math.TransformPoint(transformB, baseSim.LocalOriginAnchorB);

            Vec2 axis = B2Math.Sub(pB, pA).Normalize;

            if (joint.MinLength < joint.MaxLength && joint.EnableLimit)
            {
                Vec2 pMin = B2Math.MulAdd(pA, joint.MinLength, axis);
                Vec2 pMax = B2Math.MulAdd(pA, joint.MaxLength, axis);
                Vec2 offset = B2Math.MulSV(0.05f * Core.LengthUnitsPerMeter, B2Math.RightPerp(axis));

                if (joint.MinLength > Core.LinearSlop)
                {
                    // draw.DrawPoint(pMin, 4.0f, c2, draw.context);
                    draw.DrawSegment(B2Math.Sub(pMin, offset), B2Math.Add(pMin, offset), B2HexColor.LightGreen, draw.Context);
                }

                if (joint.MaxLength < Core.Huge)
                {
                    // draw.DrawPoint(pMax, 4.0f, c3, draw.context);
                    draw.DrawSegment(B2Math.Sub(pMax, offset), B2Math.Add(pMax, offset), B2HexColor.Red, draw.Context);
                }

                if (joint.MinLength > Core.LinearSlop && joint.MaxLength < Core.Huge)
                {
                    draw.DrawSegment(pMin, pMax, B2HexColor.Gray, draw.Context);
                }
            }

            draw.DrawSegment(pA, pB, B2HexColor.White, draw.Context);
            draw.DrawPoint(pA, 4.0f, B2HexColor.White, draw.Context);
            draw.DrawPoint(pB, 4.0f, B2HexColor.White, draw.Context);

            if (joint.Hertz > 0.0f && joint.EnableSpring)
            {
                Vec2 pRest = B2Math.MulAdd(pA, joint.Length, axis);
                draw.DrawPoint(pRest, 4.0f, B2HexColor.Blue, draw.Context);
            }
        }
    }
}