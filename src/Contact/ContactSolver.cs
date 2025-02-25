using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using FloatW = System.Numerics.Vector<float>;

namespace Box2DSharp
{
    // Wide vec2
    public struct Vec2W
    {
        public Vector<float> X;

        public Vector<float> Y;

        public Vec2W(Vector<float> x, Vector<float> y)
        {
            X = x;
            Y = y;
        }
    }

    // Wide rotation
    public struct RotW
    {
        public Vector<float> C, S;

        public RotW(Vector<float> c, Vector<float> s)
        {
            C = c;
            S = s;
        }
    }

    public struct ContactConstraintPoint
    {
        public Vec2 AnchorA;

        public Vec2 AnchorB;

        public float BaseSeparation;

        public float RelativeVelocity;

        public float NormalImpulse;

        public float TangentImpulse;

        public float MaxNormalImpulse;

        public float NormalMass;

        public float TangentMass;
    }

    public class ContactConstraint
    {
        public int IndexA;

        public int IndexB;

        public ContactConstraintPoint Point1;

        public ContactConstraintPoint Point2;

        public Vec2 Normal;

        public float InvMassA;

        public float InvMassB;

        public float InvIA;

        public float InvIB;

        public float Friction;

        public float Restitution;

        public Softness Softness;

        public int PointCount;

        public Span<ContactConstraintPoint> Points
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => MemoryMarshal.CreateSpan(ref Point1, 2);
        }
    }

    /// <summary>
    /// Soft contact constraints with sub-stepping support<br/>
    /// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles &amp; capsules)<br/>
    /// http://mmacklin.com/smallsteps.pdf<br/>
    /// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf<br/>
    /// 接触点软约束SIMD，为雅可比矩阵使用固定锚点，使其在滚动形状上有更好的行为
    /// </summary>
    public class ContactConstraintSIMD
    {
        public FixedArray8<int> IndexA;

        public FixedArray8<int> IndexB;

        public FloatW InvMassA;

        public FloatW InvMassB;

        public FloatW InvIA;

        public FloatW InvIB;

        public Vec2W Normal;

        public FloatW Friction;

        public FloatW BiasRate;

        public FloatW MassScale;

        public FloatW ImpulseScale;

        public Vec2W AnchorA1, AnchorB1;

        public FloatW NormalMass1, TangentMass1;

        public FloatW BaseSeparation1;

        public FloatW NormalImpulse1;

        public FloatW MaxNormalImpulse1;

        public FloatW TangentImpulse1;

        public Vec2W AnchorA2;

        public Vec2W AnchorB2;

        public FloatW BaseSeparation2;

        public FloatW NormalImpulse2;

        public FloatW MaxNormalImpulse2;

        public FloatW TangentImpulse2;

        public FloatW NormalMass2;

        public FloatW TangentMass2;

        public FloatW Restitution;

        public FloatW RelativeVelocity1;

        public FloatW RelativeVelocity2;
    }

    public class ContactSolver
    {
        static ContactSolver()
        {
            if (Sse.IsSupported)
            { }

            if (Avx.IsSupported)
            {
                GatherBodies = GatherBodies_Avx;
                ScatterBodies = ScatterBodies_Avx;
            }
            else
            {
                GatherBodies = GatherBodies_ForLoop;
                ScatterBodies = ScatterBodies_ForLoop;
            }
        }

        public static void PrepareOverflowContacts(in StepContext context)
        {
            World world = context.World;
            ConstraintGraph graph = context.Graph;
            ref GraphColor color = ref graph.Colors[Core.OverflowIndex];
            Span<ContactConstraint> constraints = color.OverflowConstraints.Span;
            int contactCount = color.Contacts.Count;
            Span<ContactSim> contacts = color.Contacts;
            var awakeStates = context.States;

            var bodies = world.BodyArray;

            // Stiffer for static contacts to avoid bodies getting pushed through the ground
            Softness contactSoftness = context.ContactSoftness;
            Softness staticSoftness = context.StaticSoftness;

            float warmStartScale = world.EnableWarmStarting ? 1.0f : 0.0f;

            for (int i = 0; i < contactCount; ++i)
            {
                ContactSim contactSim = contacts[i];

                ref Manifold manifold = ref contactSim.Manifold;
                int pointCount = manifold.PointCount;

                Debug.Assert(0 < pointCount && pointCount <= 2);

                int indexA = contactSim.BodySimIndexA;
                int indexB = contactSim.BodySimIndexB;

                if (Core.B2Validate)
                {
                    Body bodyA = bodies[contactSim.BodyIdA];
                    int validIndexA = bodyA.SetIndex == SolverSetType.AwakeSet ? bodyA.LocalIndex : Core.NullIndex;
                    Debug.Assert(indexA == validIndexA);

                    Body bodyB = bodies[contactSim.BodyIdB];
                    int validIndexB = bodyB.SetIndex == SolverSetType.AwakeSet ? bodyB.LocalIndex : Core.NullIndex;
                    Debug.Assert(indexB == validIndexB);
                }

                ref ContactConstraint constraint = ref constraints[i];
                constraint.IndexA = indexA;
                constraint.IndexB = indexB;
                constraint.Normal = manifold.Normal;
                constraint.Friction = contactSim.Friction;
                constraint.Restitution = contactSim.Restitution;
                constraint.PointCount = pointCount;

                Vec2 vA = Vec2.Zero;
                float wA = 0.0f;
                float mA = contactSim.InvMassA;
                float iA = contactSim.InvIA;
                if (indexA != Core.NullIndex)
                {
                    ref BodyState stateA = ref awakeStates[indexA];
                    vA = stateA.LinearVelocity;
                    wA = stateA.AngularVelocity;
                }

                Vec2 vB = Vec2.Zero;
                float wB = 0.0f;
                float mB = contactSim.InvMassB;
                float iB = contactSim.InvIB;
                if (indexB != Core.NullIndex)
                {
                    ref BodyState stateB = ref awakeStates[indexB];
                    vB = stateB.LinearVelocity;
                    wB = stateB.AngularVelocity;
                }

                if (indexA == Core.NullIndex || indexB == Core.NullIndex)
                {
                    constraint.Softness = staticSoftness;
                }
                else
                {
                    constraint.Softness = contactSoftness;
                }

                // copy mass into constraint to avoid cache misses during sub-stepping
                constraint.InvMassA = mA;
                constraint.InvIA = iA;
                constraint.InvMassB = mB;
                constraint.InvIB = iB;

                Vec2 normal = constraint.Normal;
                Vec2 tangent = B2Math.RightPerp(constraint.Normal);

                for (int j = 0; j < pointCount; ++j)
                {
                    ref ManifoldPoint mp = ref manifold.Points[j];
                    ref ContactConstraintPoint cp = ref constraint.Points[j];

                    cp.NormalImpulse = warmStartScale * mp.NormalImpulse;
                    cp.TangentImpulse = warmStartScale * mp.TangentImpulse;
                    cp.MaxNormalImpulse = 0.0f;

                    Vec2 rA = mp.AnchorA;
                    Vec2 rB = mp.AnchorB;

                    cp.AnchorA = rA;
                    cp.AnchorB = rB;
                    cp.BaseSeparation = mp.Separation - B2Math.Dot(B2Math.Sub(rB, rA), normal);

                    float rnA = B2Math.Cross(rA, normal);
                    float rnB = B2Math.Cross(rB, normal);
                    float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                    cp.NormalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                    float rtA = B2Math.Cross(rA, tangent);
                    float rtB = B2Math.Cross(rB, tangent);
                    float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                    cp.TangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                    // Save relative velocity for restitution
                    Vec2 vrA = B2Math.Add(vA, B2Math.CrossSV(wA, rA));
                    Vec2 vrB = B2Math.Add(vB, B2Math.CrossSV(wB, rB));
                    cp.RelativeVelocity = B2Math.Dot(normal, B2Math.Sub(vrB, vrA));
                }
            }
        }

        public static void WarmStartOverflowContacts(in StepContext context)
        {
            ConstraintGraph graph = context.Graph;
            GraphColor color = graph.Colors[Core.OverflowIndex];
            Span<ContactConstraint> constraints = color.OverflowConstraints.Span;
            int contactCount = color.Contacts.Count;
            ref SolverSet awakeSet = ref context.World.SolverSetArray[SolverSetType.AwakeSet];
            Span<BodyState> states = awakeSet.States;

            // This is a dummy state to represent a static body because static bodies don't have a solver body.
            BodyState dummyState = BodyState.Identity;

            for (int i = 0; i < contactCount; ++i)
            {
                ref ContactConstraint constraint = ref constraints[i];

                int indexA = constraint.IndexA;
                int indexB = constraint.IndexB;

                ref BodyState stateA = ref indexA == Core.NullIndex ? ref dummyState : ref states[indexA];
                ref BodyState stateB = ref indexB == Core.NullIndex ? ref dummyState : ref states[indexB];

                Vec2 vA = stateA.LinearVelocity;
                float wA = stateA.AngularVelocity;
                Vec2 vB = stateB.LinearVelocity;
                float wB = stateB.AngularVelocity;

                float mA = constraint.InvMassA;
                float iA = constraint.InvIA;
                float mB = constraint.InvMassB;
                float iB = constraint.InvIB;

                // Stiffer for static contacts to avoid bodies getting pushed through the ground
                Vec2 normal = constraint.Normal;
                Vec2 tangent = B2Math.RightPerp(constraint.Normal);
                int pointCount = constraint.PointCount;

                for (int j = 0; j < pointCount; ++j)
                {
                    ref ContactConstraintPoint cp = ref constraint.Points[j];

                    // fixed anchors
                    Vec2 rA = cp.AnchorA;
                    Vec2 rB = cp.AnchorB;

                    Vec2 P = B2Math.Add(B2Math.MulSV(cp.NormalImpulse, normal), B2Math.MulSV(cp.TangentImpulse, tangent));
                    wA -= iA * B2Math.Cross(rA, P);
                    vA = B2Math.MulAdd(vA, -mA, P);
                    wB += iB * B2Math.Cross(rB, P);
                    vB = B2Math.MulAdd(vB, mB, P);
                }

                stateA.LinearVelocity = vA;
                stateA.AngularVelocity = wA;
                stateB.LinearVelocity = vB;
                stateB.AngularVelocity = wB;
            }
        }

        public static void SolveOverflowContacts(in StepContext context, bool useBias)
        {
            ConstraintGraph graph = context.Graph;
            ref GraphColor color = ref graph.Colors[Core.OverflowIndex];
            Span<ContactConstraint> constraints = color.OverflowConstraints.Span;
            int contactCount = color.Contacts.Count;
            ref SolverSet awakeSet = ref context.World.SolverSetArray[SolverSetType.AwakeSet];
            Span<BodyState> states = awakeSet.States;

            float inv_h = context.InvH;
            float pushout = context.World.ContactPushOutVelocity;

            // This is a dummy body to represent a static body since static bodies don't have a solver body.
            BodyState dummyState = BodyState.Identity;

            for (int i = 0; i < contactCount; ++i)
            {
                ref ContactConstraint constraint = ref constraints[i];
                float mA = constraint.InvMassA;
                float iA = constraint.InvIA;
                float mB = constraint.InvMassB;
                float iB = constraint.InvIB;

                ref BodyState stateA = ref constraint.IndexA == Core.NullIndex ? ref dummyState : ref states[constraint.IndexA];

                Vec2 vA = stateA.LinearVelocity;
                float wA = stateA.AngularVelocity;
                Rot dqA = stateA.DeltaRotation;

                ref BodyState stateB = ref constraint.IndexB == Core.NullIndex ? ref dummyState : ref states[constraint.IndexB];

                Vec2 vB = stateB.LinearVelocity;
                float wB = stateB.AngularVelocity;
                Rot dqB = stateB.DeltaRotation;

                Vec2 dp = B2Math.Sub(stateB.DeltaPosition, stateA.DeltaPosition);

                Vec2 normal = constraint.Normal;
                Vec2 tangent = B2Math.RightPerp(normal);
                float friction = constraint.Friction;
                Softness softness = constraint.Softness;

                int pointCount = constraint.PointCount;

                for (int j = 0; j < pointCount; ++j)
                {
                    ref ContactConstraintPoint cp = ref constraint.Points[j];

                    // compute current separation
                    // this is subject to round-off error if the anchor is far from the body center of mass
                    Vec2 ds = B2Math.Add(dp, B2Math.Sub(B2Math.RotateVector(dqB, cp.AnchorB), B2Math.RotateVector(dqA, cp.AnchorA)));
                    float s = B2Math.Dot(ds, normal) + cp.BaseSeparation;

                    float velocityBias = 0.0f;
                    float massScale = 1.0f;
                    float impulseScale = 0.0f;
                    if (s > 0.0f)
                    {
                        // speculative bias
                        velocityBias = s * inv_h;
                    }
                    else if (useBias)
                    {
                        velocityBias = Math.Max(softness.BiasRate * s, -pushout);
                        massScale = softness.MassScale;
                        impulseScale = softness.ImpulseScale;
                    }

                    // fixed anchor points
                    Vec2 rA = cp.AnchorA;
                    Vec2 rB = cp.AnchorB;

                    // relative normal velocity at contact
                    Vec2 vrA = B2Math.Add(vA, B2Math.CrossSV(wA, rA));
                    Vec2 vrB = B2Math.Add(vB, B2Math.CrossSV(wB, rB));
                    float vn = B2Math.Dot(B2Math.Sub(vrB, vrA), normal);

                    // incremental normal impulse
                    float impulse = -cp.NormalMass * massScale * (vn + velocityBias) - impulseScale * cp.NormalImpulse;

                    // clamp the accumulated impulse
                    float newImpulse = Math.Max(cp.NormalImpulse + impulse, 0.0f);
                    impulse = newImpulse - cp.NormalImpulse;
                    cp.NormalImpulse = newImpulse;
                    cp.MaxNormalImpulse = Math.Max(cp.MaxNormalImpulse, impulse);

                    // apply normal impulse
                    Vec2 P = B2Math.MulSV(impulse, normal);
                    vA = B2Math.MulSub(vA, mA, P);
                    wA -= iA * B2Math.Cross(rA, P);

                    vB = B2Math.MulAdd(vB, mB, P);
                    wB += iB * B2Math.Cross(rB, P);
                }

                for (int j = 0; j < pointCount; ++j)
                {
                    ref ContactConstraintPoint cp = ref constraint.Points[j];

                    // fixed anchor points
                    Vec2 rA = cp.AnchorA;
                    Vec2 rB = cp.AnchorB;

                    // relative tangent velocity at contact
                    Vec2 vrB = B2Math.Add(vB, B2Math.CrossSV(wB, rB));
                    Vec2 vrA = B2Math.Add(vA, B2Math.CrossSV(wA, rA));
                    float vt = B2Math.Dot(B2Math.Sub(vrB, vrA), tangent);

                    // incremental tangent impulse
                    float impulse = cp.TangentMass * (-vt);

                    // clamp the accumulated force
                    float maxFriction = friction * cp.NormalImpulse;
                    float newImpulse = Math.Clamp(cp.TangentImpulse + impulse, -maxFriction, maxFriction);
                    impulse = newImpulse - cp.TangentImpulse;
                    cp.TangentImpulse = newImpulse;

                    // apply tangent impulse
                    Vec2 P = B2Math.MulSV(impulse, tangent);
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
        }

        public static void ApplyOverflowRestitution(in StepContext context)
        {
            ConstraintGraph graph = context.Graph;
            ref GraphColor color = ref graph.Colors[Core.OverflowIndex];
            Span<ContactConstraint> constraints = color.OverflowConstraints.Span;
            int contactCount = color.Contacts.Count;
            SolverSet awakeSet = context.World.SolverSetArray[SolverSetType.AwakeSet];
            Span<BodyState> states = awakeSet.States;

            float threshold = context.World.RestitutionThreshold;

            // dummy state to represent a static body
            BodyState dummyState = BodyState.Identity;

            for (int i = 0; i < contactCount; ++i)
            {
                ref ContactConstraint constraint = ref constraints[i];

                float restitution = constraint.Restitution;
                if (restitution == 0.0f)
                {
                    continue;
                }

                float mA = constraint.InvMassA;
                float iA = constraint.InvIA;
                float mB = constraint.InvMassB;
                float iB = constraint.InvIB;

                ref BodyState stateA = ref constraint.IndexA == Core.NullIndex ? ref dummyState : ref states[constraint.IndexA];
                Vec2 vA = stateA.LinearVelocity;
                float wA = stateA.AngularVelocity;

                ref BodyState stateB = ref constraint.IndexB == Core.NullIndex ? ref dummyState : ref states[constraint.IndexB];
                Vec2 vB = stateB.LinearVelocity;
                float wB = stateB.AngularVelocity;

                Vec2 normal = constraint.Normal;
                int pointCount = constraint.PointCount;

                // it is possible to get more accurate restitution by iterating
                // this only makes a difference if there are two contact points
                // for (int iter = 0; iter < 10; ++iter)
                {
                    for (int j = 0; j < pointCount; ++j)
                    {
                        ref ContactConstraintPoint cp = ref constraint.Points[j];

                        // if the normal impulse is zero then there was no collision
                        // this skips speculative contact points that didn't generate an impulse
                        // The max normal impulse is used in case there was a collision that moved away within the sub-step process
                        if (cp.RelativeVelocity > -threshold || cp.MaxNormalImpulse == 0.0f)
                        {
                            continue;
                        }

                        // fixed anchor points
                        Vec2 rA = cp.AnchorA;
                        Vec2 rB = cp.AnchorB;

                        // relative normal velocity at contact
                        Vec2 vrB = B2Math.Add(vB, B2Math.CrossSV(wB, rB));
                        Vec2 vrA = B2Math.Add(vA, B2Math.CrossSV(wA, rA));
                        float vn = B2Math.Dot(B2Math.Sub(vrB, vrA), normal);

                        // compute normal impulse
                        float impulse = -cp.NormalMass * (vn + restitution * cp.RelativeVelocity);

                        // clamp the accumulated impulse
                        // todo should this be stored?
                        float newImpulse = Math.Max(cp.NormalImpulse + impulse, 0.0f);
                        impulse = newImpulse - cp.NormalImpulse;
                        cp.NormalImpulse = newImpulse;
                        cp.MaxNormalImpulse = Math.Max(cp.MaxNormalImpulse, impulse);

                        // apply contact impulse
                        Vec2 P = B2Math.MulSV(impulse, normal);
                        vA = B2Math.MulSub(vA, mA, P);
                        wA -= iA * B2Math.Cross(rA, P);
                        vB = B2Math.MulAdd(vB, mB, P);
                        wB += iB * B2Math.Cross(rB, P);
                    }
                }

                stateA.LinearVelocity = vA;
                stateA.AngularVelocity = wA;
                stateB.LinearVelocity = vB;
                stateB.AngularVelocity = wB;
            }
        }

        public static void StoreOverflowImpulses(in StepContext context)
        {
            ConstraintGraph graph = context.Graph;
            ref GraphColor color = ref graph.Colors[Core.OverflowIndex];
            Span<ContactConstraint> constraints = color.OverflowConstraints.Span;
            Span<ContactSim> contacts = color.Contacts.Data;
            int contactCount = color.Contacts.Count;

            // float hitEventThreshold = context.world.hitEventThreshold;

            for (int i = 0; i < contactCount; ++i)
            {
                ref ContactConstraint constraint = ref constraints[i];
                ref ContactSim contact = ref contacts[i];
                ref Manifold manifold = ref contact.Manifold;
                int pointCount = manifold.PointCount;

                for (int j = 0; j < pointCount; ++j)
                {
                    manifold.Points[j].NormalImpulse = constraint.Points[j].NormalImpulse;
                    manifold.Points[j].TangentImpulse = constraint.Points[j].TangentImpulse;
                    manifold.Points[j].MaxNormalImpulse = constraint.Points[j].MaxNormalImpulse;
                    manifold.Points[j].NormalVelocity = constraint.Points[j].RelativeVelocity;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW ZeroW() => FloatW.Zero;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW SplatW(float scalar) => new(scalar);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW AddW(FloatW a, FloatW b) => a + b;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW SubW(FloatW a, FloatW b) => a - b;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW MulW(FloatW a, FloatW b) => a * b;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW MulAddW(FloatW a, FloatW b, FloatW c) => a + b * c;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW MulSubW(FloatW a, FloatW b, FloatW c) => a - b * c;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW MinW(FloatW a, FloatW b) => Vector.Min(a, b);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW MaxW(FloatW a, FloatW b) => Vector.Max(a, b);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW OrW(FloatW a, FloatW b)
        {
            var ac = Vector.Equals(a, Vector<float>.Zero) + Vector<int>.One;
            var bc = Vector.Equals(b, Vector<float>.Zero) + Vector<int>.One;
            var dc = Vector.BitwiseOr(ac, bc);
            return Vector.ConvertToSingle(dc);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW GreaterThanW(FloatW a, FloatW b)
        {
            return Vector.ConvertToSingle(Vector.GreaterThan(a, b));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW EqualsW(FloatW a, FloatW b)
        {
            return Vector.ConvertToSingle(Vector.Equals(a, b));
        }

        // component-wise returns mask ? b : a
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW BlendW(FloatW a, FloatW b, FloatW mask)
        {
            var mask2 = Vector.Multiply(mask, float.MaxValue);
            return Vector.ConditionalSelect(mask2, b, a);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW DotW(Vec2W a, Vec2W b)
        {
            return AddW(MulW(a.X, b.X), MulW(a.Y, b.Y));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FloatW CrossW(Vec2W a, Vec2W b)
        {
            return SubW(MulW(a.X, b.Y), MulW(a.Y, b.X));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vec2W RotateVectorW(RotW q, Vec2W v)
        {
            return new Vec2W(SubW(MulW(q.C, v.X), MulW(q.S, v.Y)), AddW(MulW(q.S, v.X), MulW(q.C, v.Y)));
        }

        // wide version of b2BodyState
        public struct SimdBody
        {
            public Vec2W V;

            public FloatW W;

            public FloatW Flags;

            public Vec2W Dp;

            public RotW Dq;
        }

        public delegate SimdBody GatherBodiesFunc(Span<BodyState> bodyStates, Span<int> indices);

        public static readonly GatherBodiesFunc GatherBodies;

        /// <summary>
        /// This is a load and transpose<br/>
        /// 把body的linearVelocity、angularVelocity、flag、deltaPosition、deltaRotating按SIMD宽度批量装载到SimdBody中加速计算
        /// </summary>
        /// <param name="states"></param>
        /// <param name="indices"></param>
        /// <returns></returns>
        public static SimdBody GatherBodies_Avx(Span<BodyState> states, Span<int> indices)
        {
            Vector256<float> identity = Vector256.Create(0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, 1.0f, 0.0f);
            unsafe
            {
                Vector256<float> b0 = indices[0] == Core.NullIndex ? identity : Avx.LoadVector256((float*)Unsafe.AsPointer(ref states[indices[0]]));
                Vector256<float> b1 = indices[1] == Core.NullIndex ? identity : Avx.LoadVector256((float*)Unsafe.AsPointer(ref states[indices[1]]));
                Vector256<float> b2 = indices[2] == Core.NullIndex ? identity : Avx.LoadVector256((float*)Unsafe.AsPointer(ref states[indices[2]]));
                Vector256<float> b3 = indices[3] == Core.NullIndex ? identity : Avx.LoadVector256((float*)Unsafe.AsPointer(ref states[indices[3]]));
                Vector256<float> b4 = indices[4] == Core.NullIndex ? identity : Avx.LoadVector256((float*)Unsafe.AsPointer(ref states[indices[4]]));
                Vector256<float> b5 = indices[5] == Core.NullIndex ? identity : Avx.LoadVector256((float*)Unsafe.AsPointer(ref states[indices[5]]));
                Vector256<float> b6 = indices[6] == Core.NullIndex ? identity : Avx.LoadVector256((float*)Unsafe.AsPointer(ref states[indices[6]]));
                Vector256<float> b7 = indices[7] == Core.NullIndex ? identity : Avx.LoadVector256((float*)Unsafe.AsPointer(ref states[indices[7]]));

                Vector256<float> t0 = Avx.UnpackLow(b0, b1);
                Vector256<float> t1 = Avx.UnpackHigh(b0, b1);
                Vector256<float> t2 = Avx.UnpackLow(b2, b3);
                Vector256<float> t3 = Avx.UnpackHigh(b2, b3);
                Vector256<float> t4 = Avx.UnpackLow(b4, b5);
                Vector256<float> t5 = Avx.UnpackHigh(b4, b5);
                Vector256<float> t6 = Avx.UnpackLow(b6, b7);
                Vector256<float> t7 = Avx.UnpackHigh(b6, b7);
                Vector256<float> tt0 = Avx.Shuffle(t0, t2, 0b01000100);
                Vector256<float> tt1 = Avx.Shuffle(t0, t2, 0b11101110);
                Vector256<float> tt2 = Avx.Shuffle(t1, t3, 0b01000100);
                Vector256<float> tt3 = Avx.Shuffle(t1, t3, 0b11101110);
                Vector256<float> tt4 = Avx.Shuffle(t4, t6, 0b01000100);
                Vector256<float> tt5 = Avx.Shuffle(t4, t6, 0b11101110);
                Vector256<float> tt6 = Avx.Shuffle(t5, t7, 0b01000100);
                Vector256<float> tt7 = Avx.Shuffle(t5, t7, 0b11101110);

                SimdBody simdBody = new();
                simdBody.V.X = Avx.Permute2x128(tt0, tt4, 0x20).AsVector();
                simdBody.V.Y = Avx.Permute2x128(tt1, tt5, 0x20).AsVector();
                simdBody.W = Avx.Permute2x128(tt2, tt6, 0x20).AsVector();
                simdBody.Flags = Avx.Permute2x128(tt3, tt7, 0x20).AsVector();
                simdBody.Dp.X = Avx.Permute2x128(tt0, tt4, 0x31).AsVector();
                simdBody.Dp.Y = Avx.Permute2x128(tt1, tt5, 0x31).AsVector();
                simdBody.Dq.C = Avx.Permute2x128(tt2, tt6, 0x31).AsVector();
                simdBody.Dq.S = Avx.Permute2x128(tt3, tt7, 0x31).AsVector();

                return simdBody;
            }
        }

        /// <summary>
        /// This is a load and transpose<br/>
        /// 把body的linearVelocity、angularVelocity、flag、deltaPosition、deltaRotating按SIMD宽度批量装载到SimdBody中加速计算
        /// </summary>
        /// <param name="states"></param>
        /// <param name="indices"></param>
        /// <returns></returns>
        public static SimdBody GatherBodies_ForLoop(Span<BodyState> states, Span<int> indices)
        {
            BodyState identity = BodyState.Identity;

            SimdBody simdBody = new();
            Span<float> vX = stackalloc float[indices.Length];
            Span<float> vY = stackalloc float[indices.Length];
            Span<float> w = stackalloc float[indices.Length];
            Span<float> f = stackalloc float[indices.Length];
            Span<float> dX = stackalloc float[indices.Length];
            Span<float> dY = stackalloc float[indices.Length];
            Span<float> dC = stackalloc float[indices.Length];
            Span<float> dS = stackalloc float[indices.Length];

            for (int i = 0; i < indices.Length; i++)
            {
                ref var s = ref indices[i] == Core.NullIndex ? ref identity : ref states[indices[i]];

                vX[i] = s.LinearVelocity.X;
                vY[i] = s.LinearVelocity.Y;
                w[i] = s.AngularVelocity;
                f[i] = s.Flags;
                dX[i] = s.DeltaPosition.X;
                dY[i] = s.DeltaPosition.Y;
                dC[i] = s.DeltaRotation.C;
                dS[i] = s.DeltaRotation.S;
            }

            simdBody.V.X = new FloatW(vX);
            simdBody.V.Y = new FloatW(vY);
            simdBody.W = new FloatW(w);
            simdBody.Flags = new FloatW(f);
            simdBody.Dp.X = new FloatW(dX);
            simdBody.Dp.Y = new FloatW(dY);
            simdBody.Dq.C = new FloatW(dC);
            simdBody.Dq.S = new FloatW(dS);
            return simdBody;
        }

        public delegate void ScatterBodiesFunc(Span<BodyState> bodyStates, Span<int> indices, in SimdBody simdBody);

        public static readonly ScatterBodiesFunc ScatterBodies;

        public static void ScatterBodies_Avx(Span<BodyState> bodyStates, Span<int> indices, in SimdBody simdBody)
        {
            Vector256<float> t0 = Avx.UnpackLow(simdBody.V.X.AsVector256(), simdBody.V.Y.AsVector256());
            Vector256<float> t1 = Avx.UnpackHigh(simdBody.V.X.AsVector256(), simdBody.V.Y.AsVector256());
            Vector256<float> t2 = Avx.UnpackLow(simdBody.W.AsVector256(), simdBody.Flags.AsVector256());
            Vector256<float> t3 = Avx.UnpackHigh(simdBody.W.AsVector256(), simdBody.Flags.AsVector256());
            Vector256<float> t4 = Avx.UnpackLow(simdBody.Dp.X.AsVector256(), simdBody.Dp.Y.AsVector256());
            Vector256<float> t5 = Avx.UnpackHigh(simdBody.Dp.X.AsVector256(), simdBody.Dp.Y.AsVector256());
            Vector256<float> t6 = Avx.UnpackLow(simdBody.Dq.C.AsVector256(), simdBody.Dq.S.AsVector256());
            Vector256<float> t7 = Avx.UnpackHigh(simdBody.Dq.C.AsVector256(), simdBody.Dq.S.AsVector256());
            Vector256<float> tt0 = Avx.Shuffle(t0, t2, 0b01000100);
            Vector256<float> tt1 = Avx.Shuffle(t0, t2, 0b11101110);
            Vector256<float> tt2 = Avx.Shuffle(t1, t3, 0b01000100);
            Vector256<float> tt3 = Avx.Shuffle(t1, t3, 0b11101110);
            Vector256<float> tt4 = Avx.Shuffle(t4, t6, 0b01000100);
            Vector256<float> tt5 = Avx.Shuffle(t4, t6, 0b11101110);
            Vector256<float> tt6 = Avx.Shuffle(t5, t7, 0b01000100);
            Vector256<float> tt7 = Avx.Shuffle(t5, t7, 0b11101110);

            unsafe
            {
                // I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
                // associated cache flushing.
                if (indices[0] != Core.NullIndex)
                    Avx.Store((float*)Unsafe.AsPointer(ref bodyStates[indices[0]]), Avx.Permute2x128(tt0, tt4, 0x20));
                if (indices[1] != Core.NullIndex)
                    Avx.Store((float*)Unsafe.AsPointer(ref bodyStates[indices[1]]), Avx.Permute2x128(tt1, tt5, 0x20));
                if (indices[2] != Core.NullIndex)
                    Avx.Store((float*)Unsafe.AsPointer(ref bodyStates[indices[2]]), Avx.Permute2x128(tt2, tt6, 0x20));
                if (indices[3] != Core.NullIndex)
                    Avx.Store((float*)Unsafe.AsPointer(ref bodyStates[indices[3]]), Avx.Permute2x128(tt3, tt7, 0x20));
                if (indices[4] != Core.NullIndex)
                    Avx.Store((float*)Unsafe.AsPointer(ref bodyStates[indices[4]]), Avx.Permute2x128(tt0, tt4, 0x31));
                if (indices[5] != Core.NullIndex)
                    Avx.Store((float*)Unsafe.AsPointer(ref bodyStates[indices[5]]), Avx.Permute2x128(tt1, tt5, 0x31));
                if (indices[6] != Core.NullIndex)
                    Avx.Store((float*)Unsafe.AsPointer(ref bodyStates[indices[6]]), Avx.Permute2x128(tt2, tt6, 0x31));
                if (indices[7] != Core.NullIndex)
                    Avx.Store((float*)Unsafe.AsPointer(ref bodyStates[indices[7]]), Avx.Permute2x128(tt3, tt7, 0x31));
            }
        }

        // This writes only the velocities back to the solver bodies
        public static void ScatterBodies_ForLoop(Span<BodyState> states, Span<int> indices, in SimdBody simdBody)
        {
            for (int i = 0; i < indices.Length; i++)
            {
                var j = indices[i];
                if (j != Core.NullIndex)
                {
                    ref var state = ref states[j];
                    state.LinearVelocity.X = simdBody.V.X[i];
                    state.LinearVelocity.Y = simdBody.V.Y[i];
                    state.AngularVelocity = simdBody.W[i];
                }
            }
        }

        public static void PrepareContactsTask(int startIndex, int endIndex, StepContext context)
        {
            World world = context.World;
            var contacts = context.Contacts;
            var constraints = context.SimdContactConstraints;
            var awakeStates = context.States;

            var bodies = world.BodyArray;

            // Stiffer for static contacts to avoid bodies getting pushed through the ground
            Softness contactSoftness = context.ContactSoftness;
            Softness staticSoftness = context.StaticSoftness;

            float warmStartScale = world.EnableWarmStarting ? 1.0f : 0.0f;
            Span<float> InvMassA = stackalloc float[Core.SimdWidth];
            Span<float> InvMassB = stackalloc float[Core.SimdWidth];
            Span<float> InvIA = stackalloc float[Core.SimdWidth];
            Span<float> InvIB = stackalloc float[Core.SimdWidth];
            Span<float> NormalX = stackalloc float[Core.SimdWidth];
            Span<float> NormalY = stackalloc float[Core.SimdWidth];
            Span<float> Friction = stackalloc float[Core.SimdWidth];
            Span<float> BiasRate = stackalloc float[Core.SimdWidth];
            Span<float> MassScale = stackalloc float[Core.SimdWidth];
            Span<float> ImpulseScale = stackalloc float[Core.SimdWidth];
            Span<float> AnchorA1X = stackalloc float[Core.SimdWidth];
            Span<float> AnchorA1Y = stackalloc float[Core.SimdWidth];
            Span<float> AnchorB1X = stackalloc float[Core.SimdWidth];
            Span<float> AnchorB1Y = stackalloc float[Core.SimdWidth];
            Span<float> BaseSeparation1 = stackalloc float[Core.SimdWidth];
            Span<float> NormalImpulse1 = stackalloc float[Core.SimdWidth];
            Span<float> TangentImpulse1 = stackalloc float[Core.SimdWidth];
            Span<float> MaxNormalImpulse1 = stackalloc float[Core.SimdWidth];
            Span<float> NormalMass1 = stackalloc float[Core.SimdWidth];
            Span<float> TangentMass1 = stackalloc float[Core.SimdWidth];
            Span<float> AnchorA2X = stackalloc float[Core.SimdWidth];
            Span<float> AnchorA2Y = stackalloc float[Core.SimdWidth];
            Span<float> AnchorB2X = stackalloc float[Core.SimdWidth];
            Span<float> AnchorB2Y = stackalloc float[Core.SimdWidth];
            Span<float> BaseSeparation2 = stackalloc float[Core.SimdWidth];
            Span<float> NormalImpulse2 = stackalloc float[Core.SimdWidth];
            Span<float> TangentImpulse2 = stackalloc float[Core.SimdWidth];
            Span<float> MaxNormalImpulse2 = stackalloc float[Core.SimdWidth];
            Span<float> NormalMass2 = stackalloc float[Core.SimdWidth];
            Span<float> TangentMass2 = stackalloc float[Core.SimdWidth];
            Span<float> Restitution = stackalloc float[Core.SimdWidth];
            Span<float> RelativeVelocity1 = stackalloc float[Core.SimdWidth];
            Span<float> RelativeVelocity2 = stackalloc float[Core.SimdWidth];

            for (int i = startIndex; i < endIndex; ++i)
            {
                ContactConstraintSIMD constraint = constraints[i];
                InvMassA.Clear();
                InvMassB.Clear();
                InvIA.Clear();
                InvIB.Clear();
                NormalX.Clear();
                NormalY.Clear();
                Friction.Clear();
                BiasRate.Clear();
                MassScale.Clear();
                ImpulseScale.Clear();
                AnchorA1X.Clear();
                AnchorA1Y.Clear();
                AnchorB1X.Clear();
                AnchorB1Y.Clear();
                BaseSeparation1.Clear();
                NormalImpulse1.Clear();
                TangentImpulse1.Clear();
                MaxNormalImpulse1.Clear();
                NormalMass1.Clear();
                TangentMass1.Clear();
                AnchorA2X.Clear();
                AnchorA2Y.Clear();
                AnchorB2X.Clear();
                AnchorB2Y.Clear();
                BaseSeparation2.Clear();
                NormalImpulse2.Clear();
                TangentImpulse2.Clear();
                MaxNormalImpulse2.Clear();
                NormalMass2.Clear();
                TangentMass2.Clear();
                Restitution.Clear();
                RelativeVelocity1.Clear();
                RelativeVelocity2.Clear();
                for (int j = 0; j < Core.SimdWidth; ++j)
                {
                    ContactSim contactSim = contacts.Span[Core.SimdWidth * i + j];

                    if (contactSim != null!)
                    {
                        ref Manifold manifold = ref contactSim.Manifold;

                        int indexA = contactSim.BodySimIndexA;
                        int indexB = contactSim.BodySimIndexB;

                        //#if B2_VALIDATE
                        var bodyA = bodies[contactSim.BodyIdA];
                        int validIndexA = bodyA.SetIndex == SolverSetType.AwakeSet ? bodyA.LocalIndex : Core.NullIndex;
                        var bodyB = bodies[contactSim.BodyIdB];
                        int validIndexB = bodyB.SetIndex == SolverSetType.AwakeSet ? bodyB.LocalIndex : Core.NullIndex;

                        Debug.Assert(indexA == validIndexA);
                        Debug.Assert(indexB == validIndexB);

                        //#endif
                        constraint.IndexA[j] = indexA;
                        constraint.IndexB[j] = indexB;

                        Vec2 vA = Vec2.Zero;
                        float wA = 0.0f;
                        float mA = contactSim.InvMassA;
                        float iA = contactSim.InvIA;
                        if (indexA != Core.NullIndex)
                        {
                            ref BodyState stateA = ref awakeStates[indexA];
                            vA = stateA.LinearVelocity;
                            wA = stateA.AngularVelocity;
                        }

                        Vec2 vB = Vec2.Zero;
                        float wB = 0.0f;
                        float mB = contactSim.InvMassB;
                        float iB = contactSim.InvIB;
                        if (indexB != Core.NullIndex)
                        {
                            ref BodyState stateB = ref awakeStates[indexB];
                            vB = stateB.LinearVelocity;
                            wB = stateB.AngularVelocity;
                        }

                        InvMassA[j] = mA;
                        InvMassB[j] = mB;
                        InvIA[j] = iA;
                        InvIB[j] = iB;

                        // (constraint.InvMassA)[j] = mA;
                        // (constraint.InvMassB)[j] = mB;
                        // (constraint.InvIA)[j] = iA;
                        // (constraint.InvIB)[j] = iB;

                        Softness soft = (indexA == Core.NullIndex || indexB == Core.NullIndex) ? staticSoftness : contactSoftness;

                        Vec2 normal = manifold.Normal;
                        NormalX[j] = normal.X;
                        NormalY[j] = normal.Y;
                        Friction[j] = contactSim.Friction;
                        Restitution[j] = contactSim.Restitution;
                        BiasRate[j] = soft.BiasRate;
                        MassScale[j] = soft.MassScale;
                        ImpulseScale[j] = soft.ImpulseScale;

                        // constraint.Normal.X[j] = normal.X;
                        // constraint.Normal.Y[j] = normal.Y;
                        // constraint.Friction[j] = contactSim.Friction;
                        // constraint.Restitution[j] = contactSim.Restitution;
                        // constraint.BiasRate[j] = soft.BiasRate;
                        // constraint.MassScale[j] = soft.MassScale;
                        // constraint.ImpulseScale[j] = soft.ImpulseScale;

                        Vec2 tangent = B2Math.RightPerp(normal);

                        {
                            ref ManifoldPoint mp = ref manifold.Points[0];

                            Vec2 rA = mp.AnchorA;
                            Vec2 rB = mp.AnchorB;
                            AnchorA1X[j] = rA.X;
                            AnchorA1Y[j] = rA.Y;
                            AnchorB1X[j] = rB.X;
                            AnchorB1Y[j] = rB.Y;

                            BaseSeparation1[j] = mp.Separation - B2Math.Dot(B2Math.Sub(rB, rA), normal);

                            NormalImpulse1[j] = warmStartScale * mp.NormalImpulse;
                            TangentImpulse1[j] = warmStartScale * mp.TangentImpulse;
                            MaxNormalImpulse1[j] = 0.0f;

                            // (constraint.AnchorA1.X)[j] = rA.X;
                            // (constraint.AnchorA1.Y)[j] = rA.Y;
                            // (constraint.AnchorB1.X)[j] = rB.X;
                            // (constraint.AnchorB1.Y)[j] = rB.Y;
                            //
                            // (constraint.BaseSeparation1)[j] = mp.Separation - B2Math.Dot(B2Math.Sub(rB, rA), normal);
                            //
                            // (constraint.NormalImpulse1)[j] = warmStartScale * mp.NormalImpulse;
                            // (constraint.TangentImpulse1)[j] = warmStartScale * mp.TangentImpulse;
                            // (constraint.MaxNormalImpulse1)[j] = 0.0f;

                            float rnA = B2Math.Cross(rA, normal);
                            float rnB = B2Math.Cross(rB, normal);
                            float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                            NormalMass1[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                            //(constraint.NormalMass1)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                            float rtA = B2Math.Cross(rA, tangent);
                            float rtB = B2Math.Cross(rB, tangent);
                            float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                            TangentMass1[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                            //(constraint.TangentMass1)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                            // relative velocity for restitution
                            Vec2 vrA = B2Math.Add(vA, B2Math.CrossSV(wA, rA));
                            Vec2 vrB = B2Math.Add(vB, B2Math.CrossSV(wB, rB));
                            RelativeVelocity1[j] = B2Math.Dot(normal, B2Math.Sub(vrB, vrA));

                            //(constraint.RelativeVelocity1)[j] = B2Math.Dot(normal, B2Math.Sub(vrB, vrA));
                        }

                        int pointCount = manifold.PointCount;
                        Debug.Assert(0 < pointCount && pointCount <= 2);

                        if (pointCount == 2)
                        {
                            ref ManifoldPoint mp = ref manifold.Points[1];

                            Vec2 rA = mp.AnchorA;
                            Vec2 rB = mp.AnchorB;
                            AnchorA2X[j] = rA.X;
                            AnchorA2Y[j] = rA.Y;
                            AnchorB2X[j] = rB.X;
                            AnchorB2Y[j] = rB.Y;

                            BaseSeparation2[j] = mp.Separation - B2Math.Dot(B2Math.Sub(rB, rA), normal);
                            NormalImpulse2[j] = warmStartScale * mp.NormalImpulse;
                            TangentImpulse2[j] = warmStartScale * mp.TangentImpulse;
                            MaxNormalImpulse2[j] = 0.0f;

                            // (constraint.AnchorA2.X)[j] = rA.X;
                            // (constraint.AnchorA2.Y)[j] = rA.Y;
                            // (constraint.AnchorB2.X)[j] = rB.X;
                            // (constraint.AnchorB2.Y)[j] = rB.Y;
                            //
                            // (constraint.BaseSeparation2)[j] = mp.Separation - B2Math.Dot(B2Math.Sub(rB, rA), normal);
                            //
                            // (constraint.NormalImpulse2)[j] = warmStartScale * mp.NormalImpulse;
                            // (constraint.TangentImpulse2)[j] = warmStartScale * mp.TangentImpulse;
                            // (constraint.MaxNormalImpulse2)[j] = 0.0f;

                            float rnA = B2Math.Cross(rA, normal);
                            float rnB = B2Math.Cross(rB, normal);
                            float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                            NormalMass2[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                            //(constraint.NormalMass2)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                            float rtA = B2Math.Cross(rA, tangent);
                            float rtB = B2Math.Cross(rB, tangent);
                            float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                            TangentMass2[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                            //(constraint.TangentMass2)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                            // relative velocity for restitution
                            Vec2 vrA = B2Math.Add(vA, B2Math.CrossSV(wA, rA));
                            Vec2 vrB = B2Math.Add(vB, B2Math.CrossSV(wB, rB));
                            RelativeVelocity2[j] = B2Math.Dot(normal, B2Math.Sub(vrB, vrA));

                            //(constraint.RelativeVelocity2)[j] = B2Math.Dot(normal, B2Math.Sub(vrB, vrA));
                        }
                        else
                        {
                            // dummy data that has no effect
                            // BaseSeparation2[j] = 0.0f;
                            // NormalImpulse2[j] = 0.0f;
                            // TangentImpulse2[j] = 0.0f;
                            // MaxNormalImpulse2[j] = 0.0f;
                            // AnchorA2X[j] = 0.0f;
                            // AnchorA2Y[j] = 0.0f;
                            // AnchorB2X[j] = 0.0f;
                            // AnchorB2Y[j] = 0.0f;
                            // NormalMass2[j] = 0.0f;
                            // TangentMass2[j] = 0.0f;
                            // RelativeVelocity2[j] = 0.0f;
                            // (constraint.BaseSeparation2)[j] = 0.0f;
                            // (constraint.NormalImpulse2)[j] = 0.0f;
                            // (constraint.TangentImpulse2)[j] = 0.0f;
                            // (constraint.MaxNormalImpulse2)[j] = 0.0f;
                            // (constraint.AnchorA2.X)[j] = 0.0f;
                            // (constraint.AnchorA2.Y)[j] = 0.0f;
                            // (constraint.AnchorB2.X)[j] = 0.0f;
                            // (constraint.AnchorB2.Y)[j] = 0.0f;
                            // (constraint.NormalMass2)[j] = 0.0f;
                            // (constraint.TangentMass2)[j] = 0.0f;
                            // (constraint.RelativeVelocity2)[j] = 0.0f;
                        }
                    }
                    else
                    {
                        // SIMD remainder
                        constraint.IndexA[j] = Core.NullIndex;
                        constraint.IndexB[j] = Core.NullIndex;

                        // (constraint.InvMassA)[j] = 0.0f;
                        // (constraint.InvMassB)[j] = 0.0f;
                        // (constraint.InvIA)[j] = 0.0f;
                        // (constraint.InvIB)[j] = 0.0f;
                        //
                        // (constraint.Normal.X)[j] = 0.0f;
                        // (constraint.Normal.Y)[j] = 0.0f;
                        // (constraint.Friction)[j] = 0.0f;
                        // (constraint.BiasRate)[j] = 0.0f;
                        // (constraint.MassScale)[j] = 0.0f;
                        // (constraint.ImpulseScale)[j] = 0.0f;
                        //
                        // (constraint.AnchorA1.X)[j] = 0.0f;
                        // (constraint.AnchorA1.Y)[j] = 0.0f;
                        // (constraint.AnchorB1.X)[j] = 0.0f;
                        // (constraint.AnchorB1.Y)[j] = 0.0f;
                        // (constraint.BaseSeparation1)[j] = 0.0f;
                        // (constraint.NormalImpulse1)[j] = 0.0f;
                        // (constraint.TangentImpulse1)[j] = 0.0f;
                        // (constraint.MaxNormalImpulse1)[j] = 0.0f;
                        // (constraint.NormalMass1)[j] = 0.0f;
                        // (constraint.TangentMass1)[j] = 0.0f;
                        //
                        // (constraint.AnchorA2.X)[j] = 0.0f;
                        // (constraint.AnchorA2.Y)[j] = 0.0f;
                        // (constraint.AnchorB2.X)[j] = 0.0f;
                        // (constraint.AnchorB2.Y)[j] = 0.0f;
                        // (constraint.BaseSeparation2)[j] = 0.0f;
                        // (constraint.NormalImpulse2)[j] = 0.0f;
                        // (constraint.TangentImpulse2)[j] = 0.0f;
                        // (constraint.MaxNormalImpulse2)[j] = 0.0f;
                        // (constraint.NormalMass2)[j] = 0.0f;
                        // (constraint.TangentMass2)[j] = 0.0f;
                        //
                        // (constraint.Restitution)[j] = 0.0f;
                        // (constraint.RelativeVelocity1)[j] = 0.0f;
                        // (constraint.RelativeVelocity2)[j] = 0.0f;
                    }
                }

                constraint.InvMassA = new Vector<float>(InvMassA);
                constraint.InvMassB = new Vector<float>(InvMassB);
                constraint.InvIA = new Vector<float>(InvIA);
                constraint.InvIB = new Vector<float>(InvIB);
                constraint.Normal.X = new Vector<float>(NormalX);
                constraint.Normal.Y = new Vector<float>(NormalY);
                constraint.Friction = new Vector<float>(Friction);
                constraint.BiasRate = new Vector<float>(BiasRate);
                constraint.MassScale = new Vector<float>(MassScale);
                constraint.ImpulseScale = new Vector<float>(ImpulseScale);
                constraint.AnchorA1.X = new Vector<float>(AnchorA1X);
                constraint.AnchorA1.Y = new Vector<float>(AnchorA1Y);
                constraint.AnchorB1.X = new Vector<float>(AnchorB1X);
                constraint.AnchorB1.Y = new Vector<float>(AnchorB1Y);
                constraint.BaseSeparation1 = new Vector<float>(BaseSeparation1);
                constraint.NormalImpulse1 = new Vector<float>(NormalImpulse1);
                constraint.TangentImpulse1 = new Vector<float>(TangentImpulse1);
                constraint.MaxNormalImpulse1 = new Vector<float>(MaxNormalImpulse1);
                constraint.NormalMass1 = new Vector<float>(NormalMass1);
                constraint.TangentMass1 = new Vector<float>(TangentMass1);
                constraint.AnchorA2.X = new Vector<float>(AnchorA2X);
                constraint.AnchorA2.Y = new Vector<float>(AnchorA2Y);
                constraint.AnchorB2.X = new Vector<float>(AnchorB2X);
                constraint.AnchorB2.Y = new Vector<float>(AnchorB2Y);
                constraint.BaseSeparation2 = new Vector<float>(BaseSeparation2);
                constraint.NormalImpulse2 = new Vector<float>(NormalImpulse2);
                constraint.TangentImpulse2 = new Vector<float>(TangentImpulse2);
                constraint.MaxNormalImpulse2 = new Vector<float>(MaxNormalImpulse2);
                constraint.NormalMass2 = new Vector<float>(NormalMass2);
                constraint.TangentMass2 = new Vector<float>(TangentMass2);
                constraint.Restitution = new Vector<float>(Restitution);
                constraint.RelativeVelocity1 = new Vector<float>(RelativeVelocity1);
                constraint.RelativeVelocity2 = new Vector<float>(RelativeVelocity2);
            }
        }

        public static void WarmStartContactsTask(int startIndex, int endIndex, StepContext context, int colorIndex)
        {
            var states = context.States;
            Span<ContactConstraintSIMD> constraints = context.Graph.Colors[colorIndex].SimdConstraints.Span;

            for (int i = startIndex; i < endIndex; ++i)
            {
                ContactConstraintSIMD c = constraints[i];
                SimdBody bA = GatherBodies(states, c.IndexA);
                SimdBody bB = GatherBodies(states, c.IndexB);

                FloatW tangentX = c.Normal.Y;
                FloatW tangentY = SubW(ZeroW(), c.Normal.X);

                {
                    // fixed anchors
                    Vec2W rA = c.AnchorA1;
                    Vec2W rB = c.AnchorB1;

                    Vec2W P;
                    P.X = AddW(MulW(c.NormalImpulse1, c.Normal.X), MulW(c.TangentImpulse1, tangentX));
                    P.Y = AddW(MulW(c.NormalImpulse1, c.Normal.Y), MulW(c.TangentImpulse1, tangentY));
                    bA.W = MulSubW(bA.W, c.InvIA, CrossW(rA, P));
                    bA.V.X = MulSubW(bA.V.X, c.InvMassA, P.X);
                    bA.V.Y = MulSubW(bA.V.Y, c.InvMassA, P.Y);
                    bB.W = MulAddW(bB.W, c.InvIB, CrossW(rB, P));
                    bB.V.X = MulAddW(bB.V.X, c.InvMassB, P.X);
                    bB.V.Y = MulAddW(bB.V.Y, c.InvMassB, P.Y);
                }

                {
                    // fixed anchors
                    Vec2W rA = c.AnchorA2;
                    Vec2W rB = c.AnchorB2;

                    Vec2W P;
                    P.X = AddW(MulW(c.NormalImpulse2, c.Normal.X), MulW(c.TangentImpulse2, tangentX));
                    P.Y = AddW(MulW(c.NormalImpulse2, c.Normal.Y), MulW(c.TangentImpulse2, tangentY));
                    bA.W = MulSubW(bA.W, c.InvIA, CrossW(rA, P));
                    bA.V.X = MulSubW(bA.V.X, c.InvMassA, P.X);
                    bA.V.Y = MulSubW(bA.V.Y, c.InvMassA, P.Y);
                    bB.W = MulAddW(bB.W, c.InvIB, CrossW(rB, P));
                    bB.V.X = MulAddW(bB.V.X, c.InvMassB, P.X);
                    bB.V.Y = MulAddW(bB.V.Y, c.InvMassB, P.Y);
                }

                ScatterBodies(states, c.IndexA, bA);
                ScatterBodies(states, c.IndexB, bB);
            }
        }

        public static void SolveContactsTask(int startIndex, int endIndex, in StepContext context, int colorIndex, bool useBias)
        {
            Span<BodyState> states = context.States;
            Span<ContactConstraintSIMD> constraints = context.Graph.Colors[colorIndex].SimdConstraints.Span;
            FloatW inv_h = SplatW(context.InvH);
            FloatW minBiasVel = SplatW(-context.World.ContactPushOutVelocity);

            for (int i = startIndex; i < endIndex; ++i)
            {
                ContactConstraintSIMD c = constraints[i];

                SimdBody bA = GatherBodies(states, c.IndexA);
                SimdBody bB = GatherBodies(states, c.IndexB);

                FloatW biasRate, massScale, impulseScale;
                if (useBias)
                {
                    biasRate = c.BiasRate;
                    massScale = c.MassScale;
                    impulseScale = c.ImpulseScale;
                }
                else
                {
                    biasRate = ZeroW();
                    massScale = SplatW(1.0f);
                    impulseScale = ZeroW();
                }

                Vec2W dp = new(SubW(bB.Dp.X, bA.Dp.X), SubW(bB.Dp.Y, bA.Dp.Y));

                // point1 non-penetration constraint
                {
                    // moving anchors for current separation
                    Vec2W rsA = RotateVectorW(bA.Dq, c.AnchorA1);
                    Vec2W rsB = RotateVectorW(bB.Dq, c.AnchorB1);

                    // compute current separation
                    // this is subject to round-off error if the anchor is far from the body center of mass
                    Vec2W ds = new(AddW(dp.X, SubW(rsB.X, rsA.X)), AddW(dp.Y, SubW(rsB.Y, rsA.Y)));
                    FloatW s = AddW(DotW(c.Normal, ds), c.BaseSeparation1);

                    // Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
                    FloatW mask = GreaterThanW(s, ZeroW());
                    FloatW specBias = MulW(s, inv_h);
                    FloatW softBias = MaxW(MulW(biasRate, s), minBiasVel);
                    FloatW bias = BlendW(softBias, specBias, mask);

                    // fixed anchors for Jacobians
                    Vec2W rA = c.AnchorA1;
                    Vec2W rB = c.AnchorB1;

                    // Relative velocity at contact
                    FloatW dvx = SubW(SubW(bB.V.X, MulW(bB.W, rB.Y)), SubW(bA.V.X, MulW(bA.W, rA.Y)));
                    FloatW dvy = SubW(AddW(bB.V.Y, MulW(bB.W, rB.X)), AddW(bA.V.Y, MulW(bA.W, rA.X)));
                    FloatW vn = AddW(MulW(dvx, c.Normal.X), MulW(dvy, c.Normal.Y));

                    // Compute normal impulse
                    FloatW negImpulse = AddW(
                        MulW(c.NormalMass1, MulW(massScale, AddW(vn, bias))),
                        MulW(impulseScale, c.NormalImpulse1));

                    // Clamp the accumulated impulse
                    FloatW newImpulse = MaxW(SubW(c.NormalImpulse1, negImpulse), ZeroW());
                    FloatW impulse = SubW(newImpulse, c.NormalImpulse1);
                    c.NormalImpulse1 = newImpulse;
                    c.MaxNormalImpulse1 = MaxW(c.MaxNormalImpulse1, newImpulse);

                    // Apply contact impulse
                    FloatW Px = MulW(impulse, c.Normal.X);
                    FloatW Py = MulW(impulse, c.Normal.Y);

                    bA.V.X = MulSubW(bA.V.X, c.InvMassA, Px);
                    bA.V.Y = MulSubW(bA.V.Y, c.InvMassA, Py);
                    bA.W = MulSubW(bA.W, c.InvIA, SubW(MulW(rA.X, Py), MulW(rA.Y, Px)));

                    bB.V.X = MulAddW(bB.V.X, c.InvMassB, Px);
                    bB.V.Y = MulAddW(bB.V.Y, c.InvMassB, Py);
                    bB.W = MulAddW(bB.W, c.InvIB, SubW(MulW(rB.X, Py), MulW(rB.Y, Px)));
                }

                // second point non-penetration constraint
                {
                    // moving anchors for current separation
                    Vec2W rsA = RotateVectorW(bA.Dq, c.AnchorA2);
                    Vec2W rsB = RotateVectorW(bB.Dq, c.AnchorB2);

                    // compute current separation
                    Vec2W ds = new(AddW(dp.X, SubW(rsB.X, rsA.X)), AddW(dp.Y, SubW(rsB.Y, rsA.Y)));
                    FloatW s = AddW(DotW(c.Normal, ds), c.BaseSeparation2);

                    FloatW mask = GreaterThanW(s, ZeroW());
                    FloatW specBias = MulW(s, inv_h);
                    FloatW softBias = MaxW(MulW(biasRate, s), minBiasVel);
                    FloatW bias = BlendW(softBias, specBias, mask);

                    // fixed anchors for Jacobians
                    Vec2W rA = c.AnchorA2;
                    Vec2W rB = c.AnchorB2;

                    // Relative velocity at contact
                    FloatW dvx = SubW(SubW(bB.V.X, MulW(bB.W, rB.Y)), SubW(bA.V.X, MulW(bA.W, rA.Y)));
                    FloatW dvy = SubW(AddW(bB.V.Y, MulW(bB.W, rB.X)), AddW(bA.V.Y, MulW(bA.W, rA.X)));
                    FloatW vn = AddW(MulW(dvx, c.Normal.X), MulW(dvy, c.Normal.Y));

                    // Compute normal impulse
                    FloatW negImpulse = AddW(
                        MulW(c.NormalMass2, MulW(massScale, AddW(vn, bias))),
                        MulW(impulseScale, c.NormalImpulse2));

                    // Clamp the accumulated impulse
                    FloatW newImpulse = MaxW(SubW(c.NormalImpulse2, negImpulse), ZeroW());
                    FloatW impulse = SubW(newImpulse, c.NormalImpulse2);
                    c.NormalImpulse2 = newImpulse;
                    c.MaxNormalImpulse2 = MaxW(c.MaxNormalImpulse2, newImpulse);

                    // Apply contact impulse
                    FloatW Px = MulW(impulse, c.Normal.X);
                    FloatW Py = MulW(impulse, c.Normal.Y);

                    bA.V.X = MulSubW(bA.V.X, c.InvMassA, Px);
                    bA.V.Y = MulSubW(bA.V.Y, c.InvMassA, Py);
                    bA.W = MulSubW(bA.W, c.InvIA, SubW(MulW(rA.X, Py), MulW(rA.Y, Px)));

                    bB.V.X = MulAddW(bB.V.X, c.InvMassB, Px);
                    bB.V.Y = MulAddW(bB.V.Y, c.InvMassB, Py);
                    bB.W = MulAddW(bB.W, c.InvIB, SubW(MulW(rB.X, Py), MulW(rB.Y, Px)));
                }

                FloatW tangentX = c.Normal.Y;
                FloatW tangentY = SubW(ZeroW(), c.Normal.X);

                // point 1 friction constraint
                {
                    // fixed anchors for Jacobians
                    Vec2W rA = c.AnchorA1;
                    Vec2W rB = c.AnchorB1;

                    // Relative velocity at contact
                    FloatW dvx = SubW(SubW(bB.V.X, MulW(bB.W, rB.Y)), SubW(bA.V.X, MulW(bA.W, rA.Y)));
                    FloatW dvy = SubW(AddW(bB.V.Y, MulW(bB.W, rB.X)), AddW(bA.V.Y, MulW(bA.W, rA.X)));
                    FloatW vt = AddW(MulW(dvx, tangentX), MulW(dvy, tangentY));

                    // Compute tangent force
                    FloatW negImpulse = MulW(c.TangentMass1, vt);

                    // Clamp the accumulated force
                    FloatW maxFriction = MulW(c.Friction, c.NormalImpulse1);
                    FloatW newImpulse = SubW(c.TangentImpulse1, negImpulse);
                    newImpulse = MaxW(SubW(ZeroW(), maxFriction), MinW(newImpulse, maxFriction));
                    FloatW impulse = SubW(newImpulse, c.TangentImpulse1);
                    c.TangentImpulse1 = newImpulse;

                    // Apply contact impulse
                    FloatW Px = MulW(impulse, tangentX);
                    FloatW Py = MulW(impulse, tangentY);

                    bA.V.X = MulSubW(bA.V.X, c.InvMassA, Px);
                    bA.V.Y = MulSubW(bA.V.Y, c.InvMassA, Py);
                    bA.W = MulSubW(bA.W, c.InvIA, SubW(MulW(rA.X, Py), MulW(rA.Y, Px)));

                    bB.V.X = MulAddW(bB.V.X, c.InvMassB, Px);
                    bB.V.Y = MulAddW(bB.V.Y, c.InvMassB, Py);
                    bB.W = MulAddW(bB.W, c.InvIB, SubW(MulW(rB.X, Py), MulW(rB.Y, Px)));
                }

                // second point friction constraint
                {
                    // fixed anchors for Jacobians
                    Vec2W rA = c.AnchorA2;
                    Vec2W rB = c.AnchorB2;

                    // Relative velocity at contact
                    FloatW dvx = SubW(SubW(bB.V.X, MulW(bB.W, rB.Y)), SubW(bA.V.X, MulW(bA.W, rA.Y)));
                    FloatW dvy = SubW(AddW(bB.V.Y, MulW(bB.W, rB.X)), AddW(bA.V.Y, MulW(bA.W, rA.X)));
                    FloatW vt = AddW(MulW(dvx, tangentX), MulW(dvy, tangentY));

                    // Compute tangent force
                    FloatW negImpulse = MulW(c.TangentMass2, vt);

                    // Clamp the accumulated force
                    FloatW maxFriction = MulW(c.Friction, c.NormalImpulse2);
                    FloatW newImpulse = SubW(c.TangentImpulse2, negImpulse);
                    newImpulse = MaxW(SubW(ZeroW(), maxFriction), MinW(newImpulse, maxFriction));
                    FloatW impulse = SubW(newImpulse, c.TangentImpulse2);
                    c.TangentImpulse2 = newImpulse;

                    // Apply contact impulse
                    FloatW Px = MulW(impulse, tangentX);
                    FloatW Py = MulW(impulse, tangentY);

                    bA.V.X = MulSubW(bA.V.X, c.InvMassA, Px);
                    bA.V.Y = MulSubW(bA.V.Y, c.InvMassA, Py);
                    bA.W = MulSubW(bA.W, c.InvIA, SubW(MulW(rA.X, Py), MulW(rA.Y, Px)));

                    bB.V.X = MulAddW(bB.V.X, c.InvMassB, Px);
                    bB.V.Y = MulAddW(bB.V.Y, c.InvMassB, Py);
                    bB.W = MulAddW(bB.W, c.InvIB, SubW(MulW(rB.X, Py), MulW(rB.Y, Px)));
                }

                ScatterBodies(states, c.IndexA, bA);
                ScatterBodies(states, c.IndexB, bB);
            }
        }

        public static void ApplyRestitutionTask(int startIndex, int endIndex, StepContext context, int colorIndex)
        {
            Span<BodyState> states = context.States;
            Span<ContactConstraintSIMD> constraints = context.Graph.Colors[colorIndex].SimdConstraints.Span;
            FloatW threshold = SplatW(context.World.RestitutionThreshold);
            FloatW zero = ZeroW();

            for (int i = startIndex; i < endIndex; ++i)
            {
                ref ContactConstraintSIMD c = ref constraints[i];

                SimdBody bA = GatherBodies(states, c.IndexA);
                SimdBody bB = GatherBodies(states, c.IndexB);

                // first point non-penetration constraint
                {
                    // Set effective mass to zero if restitution should not be applied
                    FloatW mask1 = GreaterThanW(AddW(c.RelativeVelocity1, threshold), zero);
                    FloatW mask2 = EqualsW(c.MaxNormalImpulse1, zero);
                    FloatW mask = OrW(mask1, mask2);
                    FloatW mass = BlendW(c.NormalMass1, zero, mask);

                    // fixed anchors for Jacobians
                    Vec2W rA = c.AnchorA1;
                    Vec2W rB = c.AnchorB1;

                    // Relative velocity at contact
                    FloatW dvx = SubW(SubW(bB.V.X, MulW(bB.W, rB.Y)), SubW(bA.V.X, MulW(bA.W, rA.Y)));
                    FloatW dvy = SubW(AddW(bB.V.Y, MulW(bB.W, rB.X)), AddW(bA.V.Y, MulW(bA.W, rA.X)));
                    FloatW vn = AddW(MulW(dvx, c.Normal.X), MulW(dvy, c.Normal.Y));

                    // Compute normal impulse
                    FloatW negImpulse = MulW(mass, AddW(vn, MulW(c.Restitution, c.RelativeVelocity1)));

                    // Clamp the accumulated impulse
                    FloatW newImpulse = MaxW(SubW(c.NormalImpulse1, negImpulse), ZeroW());
                    FloatW impulse = SubW(newImpulse, c.NormalImpulse1);
                    c.NormalImpulse1 = newImpulse;

                    // Apply contact impulse
                    FloatW Px = MulW(impulse, c.Normal.X);
                    FloatW Py = MulW(impulse, c.Normal.Y);

                    bA.V.X = MulSubW(bA.V.X, c.InvMassA, Px);
                    bA.V.Y = MulSubW(bA.V.Y, c.InvMassA, Py);
                    bA.W = MulSubW(bA.W, c.InvIA, SubW(MulW(rA.X, Py), MulW(rA.Y, Px)));

                    bB.V.X = MulAddW(bB.V.X, c.InvMassB, Px);
                    bB.V.Y = MulAddW(bB.V.Y, c.InvMassB, Py);
                    bB.W = MulAddW(bB.W, c.InvIB, SubW(MulW(rB.X, Py), MulW(rB.Y, Px)));
                }

                // second point non-penetration constraint
                {
                    // Set effective mass to zero if restitution should not be applied
                    FloatW mask1 = GreaterThanW(AddW(c.RelativeVelocity2, threshold), zero);
                    FloatW mask2 = EqualsW(c.MaxNormalImpulse2, zero);
                    FloatW mask = OrW(mask1, mask2);
                    FloatW mass = BlendW(c.NormalMass2, zero, mask);

                    // fixed anchors for Jacobians
                    Vec2W rA = c.AnchorA2;
                    Vec2W rB = c.AnchorB2;

                    // Relative velocity at contact
                    FloatW dvx = SubW(SubW(bB.V.X, MulW(bB.W, rB.Y)), SubW(bA.V.X, MulW(bA.W, rA.Y)));
                    FloatW dvy = SubW(AddW(bB.V.Y, MulW(bB.W, rB.X)), AddW(bA.V.Y, MulW(bA.W, rA.X)));
                    FloatW vn = AddW(MulW(dvx, c.Normal.X), MulW(dvy, c.Normal.Y));

                    // Compute normal impulse
                    FloatW negImpulse = MulW(mass, AddW(vn, MulW(c.Restitution, c.RelativeVelocity2)));

                    // Clamp the accumulated impulse
                    FloatW newImpulse = MaxW(SubW(c.NormalImpulse2, negImpulse), ZeroW());
                    FloatW impulse = SubW(newImpulse, c.NormalImpulse2);
                    c.NormalImpulse2 = newImpulse;

                    // Apply contact impulse
                    FloatW Px = MulW(impulse, c.Normal.X);
                    FloatW Py = MulW(impulse, c.Normal.Y);

                    bA.V.X = MulSubW(bA.V.X, c.InvMassA, Px);
                    bA.V.Y = MulSubW(bA.V.Y, c.InvMassA, Py);
                    bA.W = MulSubW(bA.W, c.InvIA, SubW(MulW(rA.X, Py), MulW(rA.Y, Px)));

                    bB.V.X = MulAddW(bB.V.X, c.InvMassB, Px);
                    bB.V.Y = MulAddW(bB.V.Y, c.InvMassB, Py);
                    bB.W = MulAddW(bB.W, c.InvIB, SubW(MulW(rB.X, Py), MulW(rB.Y, Px)));
                }

                ScatterBodies(states, c.IndexA, bA);
                ScatterBodies(states, c.IndexB, bB);
            }
        }

        public static void StoreImpulsesTask(int startIndex, int endIndex, StepContext context)
        {
            Span<ContactSim> contacts = context.Contacts.Span;
            Span<ContactConstraintSIMD> constraints = context.SimdContactConstraints;

            Manifold dummy = new Manifold();

            for (int i = startIndex; i < endIndex; ++i)
            {
                ContactConstraintSIMD c = constraints[i];
                var normalImpulse1 = c.NormalImpulse1;
                var normalImpulse2 = c.NormalImpulse2;
                var tangentImpulse1 = c.TangentImpulse1;
                var tangentImpulse2 = c.TangentImpulse2;
                var maxNormalImpulse1 = c.MaxNormalImpulse1;
                var maxNormalImpulse2 = c.MaxNormalImpulse2;
                var normalVelocity1 = c.RelativeVelocity1;
                var normalVelocity2 = c.RelativeVelocity2;

                int @base = 8 * i;
                ref Manifold m0 = ref contacts[@base + 0] == null! ? ref dummy : ref contacts[@base + 0].Manifold;
                ref Manifold m1 = ref contacts[@base + 1] == null! ? ref dummy : ref contacts[@base + 1].Manifold;
                ref Manifold m2 = ref contacts[@base + 2] == null! ? ref dummy : ref contacts[@base + 2].Manifold;
                ref Manifold m3 = ref contacts[@base + 3] == null! ? ref dummy : ref contacts[@base + 3].Manifold;
                ref Manifold m4 = ref contacts[@base + 4] == null! ? ref dummy : ref contacts[@base + 4].Manifold;
                ref Manifold m5 = ref contacts[@base + 5] == null! ? ref dummy : ref contacts[@base + 5].Manifold;
                ref Manifold m6 = ref contacts[@base + 6] == null! ? ref dummy : ref contacts[@base + 6].Manifold;
                ref Manifold m7 = ref contacts[@base + 7] == null! ? ref dummy : ref contacts[@base + 7].Manifold;

                m0.Points[0].NormalImpulse = normalImpulse1[0];
                m0.Points[0].TangentImpulse = tangentImpulse1[0];
                m0.Points[0].MaxNormalImpulse = maxNormalImpulse1[0];
                m0.Points[0].NormalVelocity = normalVelocity1[0];

                m0.Points[1].NormalImpulse = normalImpulse2[0];
                m0.Points[1].TangentImpulse = tangentImpulse2[0];
                m0.Points[1].MaxNormalImpulse = maxNormalImpulse2[0];
                m0.Points[1].NormalVelocity = normalVelocity2[0];

                m1.Points[0].NormalImpulse = normalImpulse1[1];
                m1.Points[0].TangentImpulse = tangentImpulse1[1];
                m1.Points[0].MaxNormalImpulse = maxNormalImpulse1[1];
                m1.Points[0].NormalVelocity = normalVelocity1[1];

                m1.Points[1].NormalImpulse = normalImpulse2[1];
                m1.Points[1].TangentImpulse = tangentImpulse2[1];
                m1.Points[1].MaxNormalImpulse = maxNormalImpulse2[1];
                m1.Points[1].NormalVelocity = normalVelocity2[1];

                m2.Points[0].NormalImpulse = normalImpulse1[2];
                m2.Points[0].TangentImpulse = tangentImpulse1[2];
                m2.Points[0].MaxNormalImpulse = maxNormalImpulse1[2];
                m2.Points[0].NormalVelocity = normalVelocity1[2];

                m2.Points[1].NormalImpulse = normalImpulse2[2];
                m2.Points[1].TangentImpulse = tangentImpulse2[2];
                m2.Points[1].MaxNormalImpulse = maxNormalImpulse2[2];
                m2.Points[1].NormalVelocity = normalVelocity2[2];

                m3.Points[0].NormalImpulse = normalImpulse1[3];
                m3.Points[0].TangentImpulse = tangentImpulse1[3];
                m3.Points[0].MaxNormalImpulse = maxNormalImpulse1[3];
                m3.Points[0].NormalVelocity = normalVelocity1[3];

                m3.Points[1].NormalImpulse = normalImpulse2[3];
                m3.Points[1].TangentImpulse = tangentImpulse2[3];
                m3.Points[1].MaxNormalImpulse = maxNormalImpulse2[3];
                m3.Points[1].NormalVelocity = normalVelocity2[3];

                m4.Points[0].NormalImpulse = normalImpulse1[4];
                m4.Points[0].TangentImpulse = tangentImpulse1[4];
                m4.Points[0].MaxNormalImpulse = maxNormalImpulse1[4];
                m4.Points[0].NormalVelocity = normalVelocity1[4];

                m4.Points[1].NormalImpulse = normalImpulse2[4];
                m4.Points[1].TangentImpulse = tangentImpulse2[4];
                m4.Points[1].MaxNormalImpulse = maxNormalImpulse2[4];
                m4.Points[1].NormalVelocity = normalVelocity2[4];

                m5.Points[0].NormalImpulse = normalImpulse1[5];
                m5.Points[0].TangentImpulse = tangentImpulse1[5];
                m5.Points[0].MaxNormalImpulse = maxNormalImpulse1[5];
                m5.Points[0].NormalVelocity = normalVelocity1[5];

                m5.Points[1].NormalImpulse = normalImpulse2[5];
                m5.Points[1].TangentImpulse = tangentImpulse2[5];
                m5.Points[1].MaxNormalImpulse = maxNormalImpulse2[5];
                m5.Points[1].NormalVelocity = normalVelocity2[5];

                m6.Points[0].NormalImpulse = normalImpulse1[6];
                m6.Points[0].TangentImpulse = tangentImpulse1[6];
                m6.Points[0].MaxNormalImpulse = maxNormalImpulse1[6];
                m6.Points[0].NormalVelocity = normalVelocity1[6];

                m6.Points[1].NormalImpulse = normalImpulse2[6];
                m6.Points[1].TangentImpulse = tangentImpulse2[6];
                m6.Points[1].MaxNormalImpulse = maxNormalImpulse2[6];
                m6.Points[1].NormalVelocity = normalVelocity2[6];

                m7.Points[0].NormalImpulse = normalImpulse1[7];
                m7.Points[0].TangentImpulse = tangentImpulse1[7];
                m7.Points[0].MaxNormalImpulse = maxNormalImpulse1[7];
                m7.Points[0].NormalVelocity = normalVelocity1[7];

                m7.Points[1].NormalImpulse = normalImpulse2[7];
                m7.Points[1].TangentImpulse = tangentImpulse2[7];
                m7.Points[1].MaxNormalImpulse = maxNormalImpulse2[7];
                m7.Points[1].NormalVelocity = normalVelocity2[7];
            }
        }
    }
}