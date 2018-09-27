using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Collision.Collider;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    public class ContactSolver
    {
        public static bool g_blockSolve = true;

        private readonly ContactPositionConstraint[] m_positionConstraints;

        public Contact[] m_contacts;

        public int m_count;

        public Position[] m_positions;

        public TimeStep m_step;

        public Velocity[] m_velocities;

        public ContactVelocityConstraint[] VelocityConstraints;

        public ContactSolver(ContactSolverDef def)
        {
            m_step                = def.step;
            m_count               = def.count;
            m_positionConstraints = new ContactPositionConstraint[m_count];
            VelocityConstraints   = new ContactVelocityConstraint[m_count];
            for (var i = 0; i < m_count; i++)
            {
                m_positionConstraints[i] = new ContactPositionConstraint();
                VelocityConstraints[i]   = new ContactVelocityConstraint();
            }

            m_positions  = def.positions;
            m_velocities = def.velocities;
            m_contacts   = def.contacts;

            // Initialize position independent portions of the constraints.
            for (var i = 0; i < m_count; ++i)
            {
                var contact = m_contacts[i];

                var fixtureA = contact.FixtureA;
                var fixtureB = contact.FixtureB;
                var shapeA   = fixtureA.GetShape();
                var shapeB   = fixtureB.GetShape();
                var radiusA  = shapeA.Radius;
                var radiusB  = shapeB.Radius;
                var bodyA    = fixtureA.GetBody();
                var bodyB    = fixtureB.GetBody();
                var manifold = contact.GetManifold();

                var pointCount = manifold.PointCount;
                Debug.Assert(pointCount > 0);

                var vc = VelocityConstraints[i];
                vc.friction     = contact.Friction;
                vc.restitution  = contact.Restitution;
                vc.tangentSpeed = contact.TangentSpeed;
                vc.indexA       = bodyA._islandIndex;
                vc.indexB       = bodyB._islandIndex;
                vc.invMassA     = bodyA._invMass;
                vc.invMassB     = bodyB._invMass;
                vc.invIA        = bodyA._inverseInertia;
                vc.invIB        = bodyB._inverseInertia;
                vc.contactIndex = i;
                vc.pointCount   = pointCount;
                vc.K.SetZero();
                vc.normalMass.SetZero();

                var pc = m_positionConstraints[i];
                pc.indexA       = bodyA._islandIndex;
                pc.indexB       = bodyB._islandIndex;
                pc.invMassA     = bodyA._invMass;
                pc.invMassB     = bodyB._invMass;
                pc.localCenterA = bodyA._sweep.localCenter;
                pc.localCenterB = bodyB._sweep.localCenter;
                pc.invIA        = bodyA._inverseInertia;
                pc.invIB        = bodyB._inverseInertia;
                pc.localNormal  = manifold.LocalNormal;
                pc.localPoint   = manifold.LocalPoint;
                pc.pointCount   = pointCount;
                pc.radiusA      = radiusA;
                pc.radiusB      = radiusB;
                pc.type         = manifold.Type;

                for (var j = 0; j < pointCount; ++j)
                {
                    var cp  = manifold.Points[j];
                    var vcp = vc.points[j];

                    if (m_step.warmStarting)
                    {
                        vcp.normalImpulse  = m_step.dtRatio * cp.normalImpulse;
                        vcp.tangentImpulse = m_step.dtRatio * cp.tangentImpulse;
                    }
                    else
                    {
                        vcp.normalImpulse  = 0.0f;
                        vcp.tangentImpulse = 0.0f;
                    }

                    vcp.rA.SetZero();
                    vcp.rB.SetZero();
                    vcp.normalMass   = 0.0f;
                    vcp.tangentMass  = 0.0f;
                    vcp.velocityBias = 0.0f;

                    pc.localPoints[j] = cp.localPoint;
                }
            }
        }

        public void InitializeVelocityConstraints()
        {
            for (var i = 0; i < m_count; ++i)
            {
                var vc = VelocityConstraints[i];
                var pc = m_positionConstraints[i];

                var radiusA  = pc.radiusA;
                var radiusB  = pc.radiusB;
                var manifold = m_contacts[vc.contactIndex].GetManifold();

                var indexA = vc.indexA;
                var indexB = vc.indexB;

                var mA           = vc.invMassA;
                var mB           = vc.invMassB;
                var iA           = vc.invIA;
                var iB           = vc.invIB;
                var localCenterA = pc.localCenterA;
                var localCenterB = pc.localCenterB;

                var cA = m_positions[indexA].Center;
                var aA = m_positions[indexA].Angle;
                var vA = m_velocities[indexA].v;
                var wA = m_velocities[indexA].w;

                var cB = m_positions[indexB].Center;
                var aB = m_positions[indexB].Angle;
                var vB = m_velocities[indexB].v;
                var wB = m_velocities[indexB].w;

                Debug.Assert(manifold.PointCount > 0);

                var xfA = new Transform();
                var xfB = new Transform();
                xfA.Rotation.Set(aA);
                xfB.Rotation.Set(aB);
                xfA.Position = cA - MathUtils.Mul(xfA.Rotation, localCenterA);
                xfB.Position = cB - MathUtils.Mul(xfB.Rotation, localCenterB);

                var worldManifold = WorldManifold.Create();
                worldManifold.Initialize(
                    manifold,
                    xfA,
                    radiusA,
                    xfB,
                    radiusB);

                vc.normal = worldManifold.normal;

                var pointCount = vc.pointCount;
                for (var j = 0; j < pointCount; ++j)
                {
                    var vcp = vc.points[j];

                    vcp.rA = worldManifold.points[j] - cA;
                    vcp.rB = worldManifold.points[j] - cB;

                    var rnA = MathUtils.Cross(vcp.rA, vc.normal);
                    var rnB = MathUtils.Cross(vcp.rB, vc.normal);

                    var kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    vcp.normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                    var tangent = MathUtils.Cross(vc.normal, 1.0f);

                    var rtA = MathUtils.Cross(vcp.rA, tangent);
                    var rtB = MathUtils.Cross(vcp.rB, tangent);

                    var kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

                    vcp.tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                    // Setup a velocity bias for restitution.
                    vcp.velocityBias = 0.0f;
                    var vRel = MathUtils.Dot(
                        vc.normal,
                        vB + MathUtils.Cross(wB, vcp.rB) - vA - MathUtils.Cross(wA, vcp.rA));
                    if (vRel < -Settings.VelocityThreshold)
                    {
                        vcp.velocityBias = -vc.restitution * vRel;
                    }
                }

                // If we have two points, then prepare the block solver.
                if (vc.pointCount == 2 && g_blockSolve)
                {
                    var vcp1 = vc.points[0];
                    var vcp2 = vc.points[1];

                    var rn1A = MathUtils.Cross(vcp1.rA, vc.normal);
                    var rn1B = MathUtils.Cross(vcp1.rB, vc.normal);
                    var rn2A = MathUtils.Cross(vcp2.rA, vc.normal);
                    var rn2B = MathUtils.Cross(vcp2.rB, vc.normal);

                    var k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
                    var k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
                    var k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

                    // Ensure a reasonable condition number.
                    const float maxConditionNumber = 1000.0f;
                    if (k11 * k11 < maxConditionNumber * (k11 * k22 - k12 * k12))
                    {
                        // K is safe to invert.
                        vc.K.ex.Set(k11, k12);
                        vc.K.ey.Set(k12, k22);
                        vc.normalMass = vc.K.GetInverse();
                    }
                    else
                    {
                        // The constraints are redundant, just use one.
                        // TODO_ERIN use deepest?
                        vc.pointCount = 1;
                    }
                }
            }
        }

        public void WarmStart()
        {
            // Warm start.
            for (var i = 0; i < m_count; ++i)
            {
                var vc = VelocityConstraints[i];

                var indexA     = vc.indexA;
                var indexB     = vc.indexB;
                var mA         = vc.invMassA;
                var iA         = vc.invIA;
                var mB         = vc.invMassB;
                var iB         = vc.invIB;
                var pointCount = vc.pointCount;

                var vA = m_velocities[indexA].v;
                var wA = m_velocities[indexA].w;
                var vB = m_velocities[indexB].v;
                var wB = m_velocities[indexB].w;

                var normal  = vc.normal;
                var tangent = MathUtils.Cross(normal, 1.0f);

                for (var j = 0; j < pointCount; ++j)
                {
                    var vcp = vc.points[j];
                    var P   = vcp.normalImpulse * normal + vcp.tangentImpulse * tangent;
                    wA -= iA * MathUtils.Cross(vcp.rA, P);
                    vA -= mA * P;
                    wB += iB * MathUtils.Cross(vcp.rB, P);
                    vB += mB * P;
                }

                m_velocities[indexA].v = vA;
                m_velocities[indexA].w = wA;
                m_velocities[indexB].v = vB;
                m_velocities[indexB].w = wB;
            }
        }

        public void SolveVelocityConstraints()
        {
            for (var i = 0; i < m_count; ++i)
            {
                var vc = VelocityConstraints[i];

                var indexA     = vc.indexA;
                var indexB     = vc.indexB;
                var mA         = vc.invMassA;
                var iA         = vc.invIA;
                var mB         = vc.invMassB;
                var iB         = vc.invIB;
                var pointCount = vc.pointCount;

                var vA = m_velocities[indexA].v;
                var wA = m_velocities[indexA].w;
                var vB = m_velocities[indexB].v;
                var wB = m_velocities[indexB].w;

                var normal   = vc.normal;
                var tangent  = MathUtils.Cross(normal, 1.0f);
                var friction = vc.friction;

                Debug.Assert(pointCount == 1 || pointCount == 2);

                // Solve tangent constraints first because non-penetration is more important
                // than friction.
                for (var j = 0; j < pointCount; ++j)
                {
                    var vcp = vc.points[j];

                    // Relative velocity at contact
                    var dv = vB + MathUtils.Cross(wB, vcp.rB) - vA - MathUtils.Cross(wA, vcp.rA);

                    // Compute tangent force
                    var vt     = MathUtils.Dot(dv, tangent) - vc.tangentSpeed;
                    var lambda = vcp.tangentMass * -vt;

                    // MathUtils.b2Clamp the accumulated force
                    var maxFriction = friction * vcp.normalImpulse;
                    var newImpulse  = MathUtils.Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
                    lambda             = newImpulse - vcp.tangentImpulse;
                    vcp.tangentImpulse = newImpulse;

                    // Apply contact impulse
                    var P = lambda * tangent;

                    vA -= mA * P;
                    wA -= iA * MathUtils.Cross(vcp.rA, P);

                    vB += mB * P;
                    wB += iB * MathUtils.Cross(vcp.rB, P);
                }

                // Solve normal constraints
                if (pointCount == 1 || g_blockSolve == false)
                {
                    for (var j = 0; j < pointCount; ++j)
                    {
                        var vcp = vc.points[j];

                        // Relative velocity at contact
                        var dv = vB + MathUtils.Cross(wB, vcp.rB) - vA - MathUtils.Cross(wA, vcp.rA);

                        // Compute normal impulse
                        var vn     = MathUtils.Dot(dv, normal);
                        var lambda = -vcp.normalMass * (vn - vcp.velocityBias);

                        // MathUtils.b2Clamp the accumulated impulse
                        var newImpulse = Math.Max(vcp.normalImpulse + lambda, 0.0f);
                        lambda            = newImpulse - vcp.normalImpulse;
                        vcp.normalImpulse = newImpulse;

                        // Apply contact impulse
                        var P = lambda * normal;
                        vA -= mA * P;
                        wA -= iA * MathUtils.Cross(vcp.rA, P);

                        vB += mB * P;
                        wB += iB * MathUtils.Cross(vcp.rB, P);
                    }
                }
                else
                {
                    // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
                    // Build the mini LCP for this contact patch
                    //
                    // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                    //
                    // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                    // b = vn0 - velocityBias
                    //
                    // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                    // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
                    // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                    // solution that satisfies the problem is chosen.
                    // 
                    // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                    // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                    //
                    // Substitute:
                    // 
                    // x = a + d
                    // 
                    // a := old total impulse
                    // x := new total impulse
                    // d := incremental impulse 
                    //
                    // For the current iteration we extend the formula for the incremental impulse
                    // to compute the new total impulse:
                    //
                    // vn = A * d + b
                    //    = A * (x - a) + b
                    //    = A * x + b - A * a
                    //    = A * x + b'
                    // b' = b - A * a;

                    var cp1 = vc.points[0];
                    var cp2 = vc.points[1];

                    var a = new Vector2(cp1.normalImpulse, cp2.normalImpulse);
                    Debug.Assert(a.X >= 0.0f && a.Y >= 0.0f);

                    // Relative velocity at contact
                    var dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);
                    var dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

                    // Compute normal velocity
                    var vn1 = MathUtils.Dot(dv1, normal);
                    var vn2 = MathUtils.Dot(dv2, normal);

                    var b = new Vector2();
                    b.X = vn1 - cp1.velocityBias;
                    b.Y = vn2 - cp2.velocityBias;

                    // Compute b'
                    b -= MathUtils.Mul(vc.K, a);

                    for (;;)
                    {
                        //
                        // Case 1: vn = 0
                        //
                        // 0 = A * x + b'
                        //
                        // Solve for x:
                        //
                        // x = - inv(A) * b'
                        //
                        var x = -MathUtils.Mul(vc.normalMass, b);

                        if (x.X >= 0.0f && x.Y >= 0.0f)
                        {
                            // Get the incremental impulse
                            var d = x - a;

                            // Apply incremental impulse
                            var P1 = d.X * normal;
                            var P2 = d.Y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.normalImpulse = x.X;
                            cp2.normalImpulse = x.Y;

#if B2_DEBUG_SOLVER

// Postconditions
                            const float k_errorTol = 1e-3f;
                            dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);
                            dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

                            // Compute normal velocity
                            vn1 = MathUtils.Dot(dv1, normal);
                            vn2 = MathUtils.Dot(dv2, normal);

                            Debug.Assert(Math.Abs(vn1 - cp1.velocityBias) < k_errorTol);
                            Debug.Assert(Math.Abs(vn2 - cp2.velocityBias) < k_errorTol);
#endif
                            break;
                        }

                        //
                        // Case 2: vn1 = 0 and x2 = 0
                        //
                        //   0 = a11 * x1 + a12 * 0 + b1' 
                        // vn2 = a21 * x1 + a22 * 0 + b2'
                        //
                        x.X = -cp1.normalMass * b.X;
                        x.Y = 0.0f;
                        vn1 = 0.0f;
                        vn2 = vc.K.ex.Y * x.X + b.Y;
                        if (x.X >= 0.0f && vn2 >= 0.0f)
                        {
                            // Get the incremental impulse
                            var d = x - a;

                            // Apply incremental impulse
                            var P1 = d.X * normal;
                            var P2 = d.Y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.normalImpulse = x.X;
                            cp2.normalImpulse = x.Y;

#if B2_DEBUG_SOLVER

// Postconditions
                            dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);

                            // Compute normal velocity
                            vn1 = MathUtils.Dot(dv1, normal);

                            Debug.Assert(Math.Abs(vn1 - cp1.velocityBias) < k_errorTol);
#endif
                            break;
                        }

                        //
                        // Case 3: vn2 = 0 and x1 = 0
                        //
                        // vn1 = a11 * 0 + a12 * x2 + b1' 
                        //   0 = a21 * 0 + a22 * x2 + b2'
                        //
                        x.X = 0.0f;
                        x.Y = -cp2.normalMass * b.Y;
                        vn1 = vc.K.ey.X * x.Y + b.X;
                        vn2 = 0.0f;

                        if (x.Y >= 0.0f && vn1 >= 0.0f)
                        {
                            // Resubstitute for the incremental impulse
                            var d = x - a;

                            // Apply incremental impulse
                            var P1 = d.X * normal;
                            var P2 = d.Y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.normalImpulse = x.X;
                            cp2.normalImpulse = x.Y;

#if B2_DEBUG_SOLVER

// Postconditions
                            dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

                            // Compute normal velocity
                            vn2 = MathUtils.Dot(dv2, normal);

                            Debug.Assert(Math.Abs(vn2 - cp2.velocityBias) < k_errorTol);
#endif
                            break;
                        }

                        //
                        // Case 4: x1 = 0 and x2 = 0
                        // 
                        // vn1 = b1
                        // vn2 = b2;
                        x.X = 0.0f;
                        x.Y = 0.0f;
                        vn1 = b.X;
                        vn2 = b.Y;

                        if (vn1 >= 0.0f && vn2 >= 0.0f)
                        {
                            // Resubstitute for the incremental impulse
                            var d = x - a;

                            // Apply incremental impulse
                            var P1 = d.X * normal;
                            var P2 = d.Y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.normalImpulse = x.X;
                            cp2.normalImpulse = x.Y;
                        }

                        // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                        break;
                    }
                }

                m_velocities[indexA].v = vA;
                m_velocities[indexA].w = wA;
                m_velocities[indexB].v = vB;
                m_velocities[indexB].w = wB;
            }
        }

        public void StoreImpulses()
        {
            for (var i = 0; i < m_count; ++i)
            {
                var vc       = VelocityConstraints[i];
                var manifold = m_contacts[vc.contactIndex].GetManifold();

                for (var j = 0; j < vc.pointCount; ++j)
                {
                    manifold.Points[j].normalImpulse  = vc.points[j].normalImpulse;
                    manifold.Points[j].tangentImpulse = vc.points[j].tangentImpulse;
                }
            }
        }

        public bool SolvePositionConstraints()
        {
            var minSeparation = 0.0f;

            for (var i = 0; i < m_count; ++i)
            {
                var pc = m_positionConstraints[i];

                var indexA       = pc.indexA;
                var indexB       = pc.indexB;
                var localCenterA = pc.localCenterA;
                var mA           = pc.invMassA;
                var iA           = pc.invIA;
                var localCenterB = pc.localCenterB;
                var mB           = pc.invMassB;
                var iB           = pc.invIB;
                var pointCount   = pc.pointCount;

                var cA = m_positions[indexA].Center;
                var aA = m_positions[indexA].Angle;

                var cB = m_positions[indexB].Center;
                var aB = m_positions[indexB].Angle;

                // Solve normal constraints
                for (var j = 0; j < pointCount; ++j)
                {
                    var xfA = new Transform();
                    var xfB = xfA;
                    xfA.Rotation.Set(aA);
                    xfB.Rotation.Set(aB);
                    xfA.Position = cA - MathUtils.Mul(xfA.Rotation, localCenterA);
                    xfB.Position = cB - MathUtils.Mul(xfB.Rotation, localCenterB);

                    var psm = new PositionSolverManifold();
                    psm.Initialize(pc, xfA, xfB, j);
                    var normal = psm.normal;

                    var point      = psm.point;
                    var separation = psm.separation;

                    var rA = point - cA;
                    var rB = point - cB;

                    // Track max constraint error.
                    minSeparation = Math.Min(minSeparation, separation);

                    // Prevent large corrections and allow slop.
                    var C = MathUtils.Clamp(
                        Settings.Baumgarte * (separation + Settings.LinearSlop),
                        -Settings.MaxLinearCorrection,
                        0.0f);

                    // Compute the effective mass.
                    var rnA = MathUtils.Cross(rA, normal);
                    var rnB = MathUtils.Cross(rB, normal);
                    var K   = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    // Compute normal impulse
                    var impulse = K > 0.0f ? -C / K : 0.0f;

                    var P = impulse * normal;

                    cA -= mA * P;
                    aA -= iA * MathUtils.Cross(rA, P);

                    cB += mB * P;
                    aB += iB * MathUtils.Cross(rB, P);
                }

                m_positions[indexA].Center = cA;
                m_positions[indexA].Angle  = aA;

                m_positions[indexB].Center = cB;
                m_positions[indexB].Angle  = aB;
            }

            // We can't expect minSpeparation >= -b2_linearSlop because we don't
            // push the separation above -b2_linearSlop.
            return minSeparation >= -3.0f * Settings.LinearSlop;
        }

        public bool SolveTOIPositionConstraints(int toiIndexA, int toiIndexB)
        {
            var minSeparation = 0.0f;

            for (var i = 0; i < m_count; ++i)
            {
                var pc = m_positionConstraints[i];

                var indexA       = pc.indexA;
                var indexB       = pc.indexB;
                var localCenterA = pc.localCenterA;
                var localCenterB = pc.localCenterB;
                var pointCount   = pc.pointCount;

                var mA = 0.0f;
                var iA = 0.0f;
                if (indexA == toiIndexA || indexA == toiIndexB)
                {
                    mA = pc.invMassA;
                    iA = pc.invIA;
                }

                var mB = 0.0f;
                var iB = 0.0f;
                if (indexB == toiIndexA || indexB == toiIndexB)
                {
                    mB = pc.invMassB;
                    iB = pc.invIB;
                }

                var cA = m_positions[indexA].Center;
                var aA = m_positions[indexA].Angle;

                var cB = m_positions[indexB].Center;
                var aB = m_positions[indexB].Angle;

                // Solve normal constraints
                for (var j = 0; j < pointCount; ++j)
                {
                    var xfA = new Transform();
                    var xfB = new Transform();
                    xfA.Rotation.Set(aA);
                    xfB.Rotation.Set(aB);
                    xfA.Position = cA - MathUtils.Mul(xfA.Rotation, localCenterA);
                    xfB.Position = cB - MathUtils.Mul(xfB.Rotation, localCenterB);

                    var psm = new PositionSolverManifold();
                    psm.Initialize(pc, xfA, xfB, j);
                    var normal = psm.normal;

                    var point      = psm.point;
                    var separation = psm.separation;

                    var rA = point - cA;
                    var rB = point - cB;

                    // Track max constraint error.
                    minSeparation = Math.Min(minSeparation, separation);

                    // Prevent large corrections and allow slop.
                    var C = MathUtils.Clamp(
                        Settings.ToiBaugarte * (separation + Settings.LinearSlop),
                        -Settings.MaxLinearCorrection,
                        0.0f);

                    // Compute the effective mass.
                    var rnA = MathUtils.Cross(rA, normal);
                    var rnB = MathUtils.Cross(rB, normal);
                    var K   = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    // Compute normal impulse
                    var impulse = K > 0.0f ? -C / K : 0.0f;

                    var P = impulse * normal;

                    cA -= mA * P;
                    aA -= iA * MathUtils.Cross(rA, P);

                    cB += mB * P;
                    aB += iB * MathUtils.Cross(rB, P);
                }

                m_positions[indexA].Center = cA;
                m_positions[indexA].Angle  = aA;

                m_positions[indexB].Center = cB;
                m_positions[indexB].Angle  = aB;
            }

            // We can't expect minSpeparation >= -b2_linearSlop because we don't
            // push the separation above -b2_linearSlop.
            return minSeparation >= -1.5f * Settings.LinearSlop;
        }
    }
}