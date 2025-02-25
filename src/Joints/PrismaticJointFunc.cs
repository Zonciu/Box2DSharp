using System;
using System.Diagnostics;

namespace Box2DSharp
{
    public class PrismaticJointFunc
    {
        public static void EnableSpring(JointId jointId, bool enableSpring)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            if (enableSpring != joint.Joint.PrismaticJoint.EnableSpring)
            {
                joint.Joint.PrismaticJoint.EnableSpring = enableSpring;
                joint.Joint.PrismaticJoint.SpringImpulse = 0.0f;
            }
        }

        public static bool IsSpringEnabled(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            return joint.Joint.PrismaticJoint.EnableSpring;
        }

        public static void SetSpringHertz(JointId jointId, float hertz)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            joint.Joint.PrismaticJoint.Hertz = hertz;
        }

        public static float GetSpringHertz(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            return joint.Joint.PrismaticJoint.Hertz;
        }

        public static void SetSpringDampingRatio(JointId jointId, float dampingRatio)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            joint.Joint.PrismaticJoint.DampingRatio = dampingRatio;
        }

        public static float GetSpringDampingRatio(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            return joint.Joint.PrismaticJoint.DampingRatio;
        }

        public static void EnableLimit(JointId jointId, bool enableLimit)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            if (enableLimit != joint.Joint.PrismaticJoint.EnableLimit)
            {
                joint.Joint.PrismaticJoint.EnableLimit = enableLimit;
                joint.Joint.PrismaticJoint.LowerImpulse = 0.0f;
                joint.Joint.PrismaticJoint.UpperImpulse = 0.0f;
            }
        }

        public static bool IsLimitEnabled(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            return joint.Joint.PrismaticJoint.EnableLimit;
        }

        public static float GetLowerLimit(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            return joint.Joint.PrismaticJoint.LowerTranslation;
        }

        public static float GetUpperLimit(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            return joint.Joint.PrismaticJoint.UpperTranslation;
        }

        public static void SetLimits(JointId jointId, float lower, float upper)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            if (lower != joint.Joint.PrismaticJoint.LowerTranslation || upper != joint.Joint.PrismaticJoint.UpperTranslation)
            {
                joint.Joint.PrismaticJoint.LowerTranslation = Math.Min(lower, upper);
                joint.Joint.PrismaticJoint.UpperTranslation = Math.Max(lower, upper);
                joint.Joint.PrismaticJoint.LowerImpulse = 0.0f;
                joint.Joint.PrismaticJoint.UpperImpulse = 0.0f;
            }
        }

        public static void EnableMotor(JointId jointId, bool enableMotor)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            if (enableMotor != joint.Joint.PrismaticJoint.EnableMotor)
            {
                joint.Joint.PrismaticJoint.EnableMotor = enableMotor;
                joint.Joint.PrismaticJoint.MotorImpulse = 0.0f;
            }
        }

        public static bool IsMotorEnabled(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            return joint.Joint.PrismaticJoint.EnableMotor;
        }

        public static void SetMotorSpeed(JointId jointId, float motorSpeed)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            joint.Joint.PrismaticJoint.MotorSpeed = motorSpeed;
        }

        public static float GetMotorSpeed(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            return joint.Joint.PrismaticJoint.MotorSpeed;
        }

        public static float GetMotorForce(JointId jointId)
        {
            World world = World.GetWorld(jointId.World0);
            JointSim simBase = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            return world.InvH * simBase.Joint.PrismaticJoint.MotorImpulse;
        }

        public static void SetMaxMotorForce(JointId jointId, float force)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            joint.Joint.PrismaticJoint.MaxMotorForce = force;
        }

        public static float GetMaxMotorForce(JointId jointId)
        {
            JointSim joint = Joint.GetJointSimCheckType(jointId, JointType.PrismaticJoint);
            return joint.Joint.PrismaticJoint.MaxMotorForce;
        }

        public static Vec2 GetPrismaticJointForce(World world, JointSim simBase)
        {
            int idA = simBase.BodyIdA;
            Transform transformA = Body.GetBodyTransform(world, idA);

            PrismaticJoint joint = simBase.Joint.PrismaticJoint;

            Vec2 axisA = B2Math.RotateVector(transformA.Q, joint.LocalAxisA);
            Vec2 perpA = B2Math.LeftPerp(axisA);

            float inv_h = world.InvH;
            float perpForce = inv_h * joint.Impulse.X;
            float axialForce = inv_h * (joint.MotorImpulse + joint.LowerImpulse - joint.UpperImpulse);

            Vec2 force = B2Math.Add(B2Math.MulSV(perpForce, perpA), B2Math.MulSV(axialForce, axisA));
            return force;
        }

        public static float GetPrismaticJointTorque(World world, JointSim simBase)
        {
            return world.InvH * simBase.Joint.PrismaticJoint.Impulse.Y;
        }

        // Linear constraint (point-to-line)
        // d = p2 - p1 = x2 + r2 - x1 - r1
        // C = dot(perp, d)
        // Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
        //      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
        // J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
        //
        // Angular constraint
        // C = a2 - a1 + a_initial
        // Cdot = w2 - w1
        // J = [0 0 -1 0 0 1]
        //
        // K = J * invM * JT
        //
        // J = [-a -s1 a s2]
        //     [0  -1  0  1]
        // a = perp
        // s1 = cross(d + r1, a) = cross(p2 - x1, a)
        // s2 = cross(r2, a) = cross(p2 - x2, a)

        // Motor/Limit linear constraint
        // C = dot(ax1, d)
        // Cdot = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
        // J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

        // Predictive limit is applied even when the limit is not active.
        // Prevents a constraint speed that can lead to a constraint error in one time step.
        // Want C2 = C1 + h * Cdot >= 0
        // Or:
        // Cdot + C1/h >= 0
        // I do not apply a negative constraint error because that is handled in position correction.
        // So:
        // Cdot + max(C1, 0)/h >= 0

        // Block Solver
        // We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
        //
        // The Jacobian has 2 rows:
        // J = [-uT -s1 uT s2] // linear
        //     [0   -1   0  1] // angular
        //
        // u = perp
        // s1 = cross(d + r1, u), s2 = cross(r2, u)
        // a1 = cross(d + r1, v), a2 = cross(r2, v)

        public static void PreparePrismaticJoint(JointSim simBase, StepContext context)
        {
            Debug.Assert(simBase.Type == JointType.PrismaticJoint);

            // chase body id to the solver set where the body lives
            int idA = simBase.BodyIdA;
            int idB = simBase.BodyIdB;

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

            simBase.InvMassA = mA;
            simBase.InvMassB = mB;
            simBase.InvIA = iA;
            simBase.InvIB = iB;

            PrismaticJoint joint = simBase.Joint.PrismaticJoint;
            joint.IndexA = bodyA.SetIndex == SolverSetType.AwakeSet ? localIndexA : Core.NullIndex;
            joint.IndexB = bodyB.SetIndex == SolverSetType.AwakeSet ? localIndexB : Core.NullIndex;

            Rot qA = bodySimA.Transform.Q;
            Rot qB = bodySimB.Transform.Q;

            joint.AnchorA = B2Math.RotateVector(qA, B2Math.Sub(simBase.LocalOriginAnchorA, bodySimA.LocalCenter));
            joint.AnchorB = B2Math.RotateVector(qB, B2Math.Sub(simBase.LocalOriginAnchorB, bodySimB.LocalCenter));
            joint.AxisA = B2Math.RotateVector(qA, joint.LocalAxisA);
            joint.DeltaCenter = B2Math.Sub(bodySimB.Center, bodySimA.Center);
            joint.DeltaAngle = B2Math.RelativeAngle(qB, qA) - joint.ReferenceAngle;

            Vec2 rA = joint.AnchorA;
            Vec2 rB = joint.AnchorB;

            Vec2 d = B2Math.Add(joint.DeltaCenter, B2Math.Sub(rB, rA));
            float a1 = B2Math.Cross(B2Math.Add(d, rA), joint.AxisA);
            float a2 = B2Math.Cross(rB, joint.AxisA);

            // effective masses
            float k = mA + mB + iA * a1 * a1 + iB * a2 * a2;
            joint.AxialMass = k > 0.0f ? 1.0f / k : 0.0f;

            joint.SpringSoftness = Softness.MakeSoft(joint.Hertz, joint.DampingRatio, context.H);

            if (context.EnableWarmStarting == false)
            {
                joint.Impulse = Vec2.Zero;
                joint.SpringImpulse = 0.0f;
                joint.MotorImpulse = 0.0f;
                joint.LowerImpulse = 0.0f;
                joint.UpperImpulse = 0.0f;
            }
        }

        public static void WarmStartPrismaticJoint(JointSim simBase, StepContext context)
        {
            Debug.Assert(simBase.Type == JointType.PrismaticJoint);

            float mA = simBase.InvMassA;
            float mB = simBase.InvMassB;
            float iA = simBase.InvIA;
            float iB = simBase.InvIB;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            PrismaticJoint joint = simBase.Joint.PrismaticJoint;

            ref BodyState stateA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState stateB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 rA = B2Math.RotateVector(stateA.DeltaRotation, joint.AnchorA);
            Vec2 rB = B2Math.RotateVector(stateB.DeltaRotation, joint.AnchorB);

            Vec2 d = B2Math.Add(B2Math.Add(B2Math.Sub(stateB.DeltaPosition, stateA.DeltaPosition), joint.DeltaCenter), B2Math.Sub(rB, rA));
            Vec2 axisA = B2Math.RotateVector(stateA.DeltaRotation, joint.AxisA);

            // impulse is applied at anchor point on body B
            float a1 = B2Math.Cross(B2Math.Add(d, rA), axisA);
            float a2 = B2Math.Cross(rB, axisA);
            float axialImpulse = joint.SpringImpulse + joint.MotorImpulse + joint.LowerImpulse - joint.UpperImpulse;

            // perpendicular constraint
            Vec2 perpA = B2Math.LeftPerp(axisA);
            float s1 = B2Math.Cross(B2Math.Add(d, rA), perpA);
            float s2 = B2Math.Cross(rB, perpA);
            float perpImpulse = joint.Impulse.X;
            float angleImpulse = joint.Impulse.Y;

            Vec2 P = B2Math.Add(B2Math.MulSV(axialImpulse, axisA), B2Math.MulSV(perpImpulse, perpA));
            float LA = axialImpulse * a1 + perpImpulse * s1 + angleImpulse;
            float LB = axialImpulse * a2 + perpImpulse * s2 + angleImpulse;

            stateA.LinearVelocity = B2Math.MulSub(stateA.LinearVelocity, mA, P);
            stateA.AngularVelocity -= iA * LA;
            stateB.LinearVelocity = B2Math.MulAdd(stateB.LinearVelocity, mB, P);
            stateB.AngularVelocity += iB * LB;
        }

        public static void SolvePrismaticJoint(JointSim simBase, StepContext context, bool useBias)
        {
            Debug.Assert(simBase.Type == JointType.PrismaticJoint);

            float mA = simBase.InvMassA;
            float mB = simBase.InvMassB;
            float iA = simBase.InvIA;
            float iB = simBase.InvIB;

            // dummy state for static bodies
            BodyState dummyState = BodyState.Identity;

            PrismaticJoint joint = simBase.Joint.PrismaticJoint;

            ref BodyState stateA = ref joint.IndexA == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexA];
            ref BodyState stateB = ref joint.IndexB == Core.NullIndex ? ref dummyState : ref context.States[joint.IndexB];

            Vec2 vA = stateA.LinearVelocity;
            float wA = stateA.AngularVelocity;
            Vec2 vB = stateB.LinearVelocity;
            float wB = stateB.AngularVelocity;

            // current anchors
            Vec2 rA = B2Math.RotateVector(stateA.DeltaRotation, joint.AnchorA);
            Vec2 rB = B2Math.RotateVector(stateB.DeltaRotation, joint.AnchorB);

            Vec2 d = B2Math.Add(B2Math.Add(B2Math.Sub(stateB.DeltaPosition, stateA.DeltaPosition), joint.DeltaCenter), B2Math.Sub(rB, rA));
            Vec2 axisA = B2Math.RotateVector(stateA.DeltaRotation, joint.AxisA);
            float translation = B2Math.Dot(axisA, d);

            // These scalars are for torques generated by axial forces
            float a1 = B2Math.Cross(B2Math.Add(d, rA), axisA);
            float a2 = B2Math.Cross(rB, axisA);

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

            // Solve motor constraint
            if (joint.EnableMotor)
            {
                float Cdot = B2Math.Dot(axisA, B2Math.Sub(vB, vA)) + a2 * wB - a1 * wA;
                float impulse = joint.AxialMass * (joint.MotorSpeed - Cdot);
                float oldImpulse = joint.MotorImpulse;
                float maxImpulse = context.H * joint.MaxMotorForce;
                joint.MotorImpulse = Math.Clamp(joint.MotorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = joint.MotorImpulse - oldImpulse;

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

                    float oldImpulse = joint.LowerImpulse;
                    float Cdot = B2Math.Dot(axisA, B2Math.Sub(vB, vA)) + a2 * wB - a1 * wA;
                    float impulse = -joint.AxialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
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

                    float oldImpulse = joint.UpperImpulse;

                    // sign flipped
                    float Cdot = B2Math.Dot(axisA, B2Math.Sub(vA, vB)) + a1 * wA - a2 * wB;
                    float impulse = -joint.AxialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
                    joint.UpperImpulse = Math.Max(oldImpulse + impulse, 0.0f);
                    impulse = joint.UpperImpulse - oldImpulse;

                    Vec2 P = B2Math.MulSV(impulse, axisA);
                    float LA = impulse * a1;
                    float LB = impulse * a2;

                    // sign flipped
                    vA = B2Math.MulAdd(vA, mA, P);
                    wA += iA * LA;
                    vB = B2Math.MulSub(vB, mB, P);
                    wB -= iB * LB;
                }
            }

            // Solve the prismatic constraint in block form
            {
                Vec2 perpA = B2Math.LeftPerp(axisA);

                // These scalars are for torques generated by the perpendicular constraint force
                float s1 = B2Math.Cross(B2Math.Add(d, rA), perpA);
                float s2 = B2Math.Cross(rB, perpA);

                Vec2 Cdot;
                Cdot.X = B2Math.Dot(perpA, B2Math.Sub(vB, vA)) + s2 * wB - s1 * wA;
                Cdot.Y = wB - wA;

                Vec2 bias = Vec2.Zero;
                float massScale = 1.0f;
                float impulseScale = 0.0f;
                if (useBias)
                {
                    Vec2 C;
                    C.X = B2Math.Dot(perpA, d);
                    C.Y = B2Math.RelativeAngle(stateB.DeltaRotation, stateA.DeltaRotation) + joint.DeltaAngle;

                    bias = B2Math.MulSV(context.JointSoftness.BiasRate, C);
                    massScale = context.JointSoftness.MassScale;
                    impulseScale = context.JointSoftness.ImpulseScale;
                }

                float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                float k12 = iA * s1 + iB * s2;
                float k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    // For bodies with fixed rotation.
                    k22 = 1.0f;
                }

                Mat22 K = new(new(k11, k12), new(k12, k22));

                Vec2 b = B2Math.Solve22(K, B2Math.Add(Cdot, bias));
                Vec2 impulse;
                impulse.X = -massScale * b.X - impulseScale * joint.Impulse.X;
                impulse.Y = -massScale * b.Y - impulseScale * joint.Impulse.Y;

                joint.Impulse.X += impulse.X;
                joint.Impulse.Y += impulse.Y;

                Vec2 P = B2Math.MulSV(impulse.X, perpA);
                float LA = impulse.X * s1 + impulse.Y;
                float LB = impulse.X * s2 + impulse.Y;

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

        public static void DrawPrismaticJoint(DebugDrawBase draw, JointSim simBase, Transform transformA, Transform transformB)
        {
            Debug.Assert(simBase.Type == JointType.PrismaticJoint);

            PrismaticJoint joint = simBase.Joint.PrismaticJoint;

            Vec2 pA = B2Math.TransformPoint(transformA, simBase.LocalOriginAnchorA);
            Vec2 pB = B2Math.TransformPoint(transformB, simBase.LocalOriginAnchorB);

            Vec2 axis = B2Math.RotateVector(transformA.Q, joint.LocalAxisA);

            B2HexColor c1 = B2HexColor.Gray7;
            B2HexColor c2 = B2HexColor.Green;
            B2HexColor c3 = B2HexColor.Red;
            B2HexColor c4 = B2HexColor.Blue;
            B2HexColor c5 = B2HexColor.Gray4;

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