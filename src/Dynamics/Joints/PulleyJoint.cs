using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// The pulley joint is connected to two bodies and two fixed ground points.
    /// The pulley supports a ratio such that:
    /// length1 + ratio * length2 <= constant
    /// Yes, the force transmitted is scaled by the ratio.
    /// Warning: the pulley joint can get a bit squirrelly by itself. They often
    /// work better when combined with prismatic joints. You should also cover the
    /// the anchor points with static shapes to prevent one side from going to
    /// zero length.
    public class PulleyJoint : Joint
    {
        /// Get the first ground anchor.
        public Vector2 GetGroundAnchorA()
        {
            return m_groundAnchorA;
        }

        /// Get the second ground anchor.
        public Vector2 GetGroundAnchorB()
        {
            return m_groundAnchorB;
        }

        /// Get the current length of the segment attached to bodyA.
        public float GetLengthA()
        {
            return m_lengthA;
        }

        /// Get the current length of the segment attached to bodyB.
        public float GetLengthB()
        {
            return m_lengthB;
        }

        /// Get the pulley ratio.
        public float GetRatio()
        {
            return m_ratio;
        }

        /// Get the current length of the segment attached to bodyA.
        public float GetCurrentLengthA()
        {
            Vector2 p = BodyA.GetWorldPoint(m_localAnchorA);
            Vector2 s = m_groundAnchorA;
            Vector2 d = p - s;
            return d.Length();
        }

        /// Get the current length of the segment attached to bodyB.
        public float GetCurrentLengthB()
        {
            Vector2 p = BodyB.GetWorldPoint(m_localAnchorB);
            Vector2 s = m_groundAnchorB;
            Vector2 d = p - s;
            return d.Length();
        }

        public PulleyJoint(PulleyJointDef def) : base(def)
        {
            m_groundAnchorA = def.groundAnchorA;
            m_groundAnchorB = def.groundAnchorB;
            m_localAnchorA  = def.localAnchorA;
            m_localAnchorB  = def.localAnchorB;

            m_lengthA = def.lengthA;
            m_lengthB = def.lengthB;

            Debug.Assert(def.ratio != 0.0f);
            m_ratio = def.ratio;

            m_constant = def.lengthA + m_ratio * def.lengthB;

            m_impulse = 0.0f;
        }

        Vector2 m_groundAnchorA;

        Vector2 m_groundAnchorB;

        float m_lengthA;

        float m_lengthB;

        // Solver shared
        Vector2 m_localAnchorA;

        Vector2 m_localAnchorB;

        float m_constant;

        float m_ratio;

        float m_impulse;

        // Solver temp
        int m_indexA;

        int m_indexB;

        Vector2 m_uA;

        Vector2 m_uB;

        Vector2 m_rA;

        Vector2 m_rB;

        Vector2 m_localCenterA;

        Vector2 m_localCenterB;

        float m_invMassA;

        float m_invMassB;

        float m_invIA;

        float m_invIB;

        float m_mass;

        /// <inheritdoc />
        public override Vector2 GetAnchorA()
        {
            return BodyA.GetWorldPoint(m_localAnchorA);
        }

        /// <inheritdoc />
        public override Vector2 GetAnchorB()
        {
            return BodyB.GetWorldPoint(m_localAnchorB);
        }

        /// <inheritdoc />
        public override Vector2 GetReactionForce(float inv_dt)
        {
            Vector2 P = m_impulse * m_uB;
            return inv_dt * P;
        }

        /// <inheritdoc />
        public override float GetReactionTorque(float inv_dt)
        {
            return 0.0f;
        }

        /// <inheritdoc />
        internal override void InitVelocityConstraints(SolverData data)
        {
            m_indexA       = BodyA._islandIndex;
            m_indexB       = BodyB._islandIndex;
            m_localCenterA = BodyA._sweep.localCenter;
            m_localCenterB = BodyB._sweep.localCenter;
            m_invMassA     = BodyA._invMass;
            m_invMassB     = BodyB._invMass;
            m_invIA        = BodyA._inverseInertia;
            m_invIB        = BodyB._inverseInertia;

            Vector2 cA = data.Positions[m_indexA].Center;
            float   aA = data.Positions[m_indexA].Angle;
            Vector2 vA = data.Velocities[m_indexA].v;
            float   wA = data.Velocities[m_indexA].w;

            Vector2 cB = data.Positions[m_indexB].Center;
            float   aB = data.Positions[m_indexB].Angle;
            Vector2 vB = data.Velocities[m_indexB].v;
            float   wB = data.Velocities[m_indexB].w;

            var qA = new Rotation(aA);
            var qB = new Rotation(aB);

            m_rA = MathUtils.Mul(qA, m_localAnchorA - m_localCenterA);
            m_rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);

            // Get the pulley axes.
            m_uA = cA + m_rA - m_groundAnchorA;
            m_uB = cB + m_rB - m_groundAnchorB;

            float lengthA = m_uA.Length();
            float lengthB = m_uB.Length();

            if (lengthA > 10.0f * Settings.LinearSlop)
            {
                m_uA *= 1.0f / lengthA;
            }
            else
            {
                m_uA.SetZero();
            }

            if (lengthB > 10.0f * Settings.LinearSlop)
            {
                m_uB *= 1.0f / lengthB;
            }
            else
            {
                m_uB.SetZero();
            }

            // Compute effective mass.
            float ruA = MathUtils.Cross(m_rA, m_uA);
            float ruB = MathUtils.Cross(m_rB, m_uB);

            float mA = m_invMassA + m_invIA * ruA * ruA;
            float mB = m_invMassB + m_invIB * ruB * ruB;

            m_mass = mA + m_ratio * m_ratio * mB;

            if (m_mass > 0.0f)
            {
                m_mass = 1.0f / m_mass;
            }

            if (data.Step.warmStarting)
            {
                // Scale impulses to support variable time steps.
                m_impulse *= data.Step.dtRatio;

                // Warm starting.
                Vector2 PA = -(m_impulse) * m_uA;
                Vector2 PB = (-m_ratio * m_impulse) * m_uB;

                vA += m_invMassA * PA;
                wA += m_invIA * MathUtils.Cross(m_rA, PA);
                vB += m_invMassB * PB;
                wB += m_invIB * MathUtils.Cross(m_rB, PB);
            }
            else
            {
                m_impulse = 0.0f;
            }

            data.Velocities[m_indexA].v = vA;
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v = vB;
            data.Velocities[m_indexB].w = wB;
        }

        /// <inheritdoc />
        internal override void SolveVelocityConstraints(SolverData data)
        {
            Vector2 vA = data.Velocities[m_indexA].v;
            float   wA = data.Velocities[m_indexA].w;
            Vector2 vB = data.Velocities[m_indexB].v;
            float   wB = data.Velocities[m_indexB].w;

            Vector2 vpA = vA + MathUtils.Cross(wA, m_rA);
            Vector2 vpB = vB + MathUtils.Cross(wB, m_rB);

            float Cdot    = -MathUtils.Dot(m_uA, vpA) - m_ratio * MathUtils.Dot(m_uB, vpB);
            float impulse = -m_mass * Cdot;
            m_impulse += impulse;

            Vector2 PA = -impulse * m_uA;
            Vector2 PB = -m_ratio * impulse * m_uB;
            vA += m_invMassA * PA;
            wA += m_invIA * MathUtils.Cross(m_rA, PA);
            vB += m_invMassB * PB;
            wB += m_invIB * MathUtils.Cross(m_rB, PB);

            data.Velocities[m_indexA].v = vA;
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v = vB;
            data.Velocities[m_indexB].w = wB;
        }

        /// <inheritdoc />
        internal override bool SolvePositionConstraints(SolverData data)
        {
            Vector2 cA = data.Positions[m_indexA].Center;
            float   aA = data.Positions[m_indexA].Angle;
            Vector2 cB = data.Positions[m_indexB].Center;
            float   aB = data.Positions[m_indexB].Angle;

            var qA = new Rotation(aA);
            var qB = new Rotation(aB);

            Vector2 rA = MathUtils.Mul(qA, m_localAnchorA - m_localCenterA);
            Vector2 rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);

            // Get the pulley axes.
            Vector2 uA = cA + rA - m_groundAnchorA;
            Vector2 uB = cB + rB - m_groundAnchorB;

            float lengthA = uA.Length();
            float lengthB = uB.Length();

            if (lengthA > 10.0f * Settings.LinearSlop)
            {
                uA *= 1.0f / lengthA;
            }
            else
            {
                uA.SetZero();
            }

            if (lengthB > 10.0f * Settings.LinearSlop)
            {
                uB *= 1.0f / lengthB;
            }
            else
            {
                uB.SetZero();
            }

            // Compute effective mass.
            float ruA = MathUtils.Cross(rA, uA);
            float ruB = MathUtils.Cross(rB, uB);

            float mA = m_invMassA + m_invIA * ruA * ruA;
            float mB = m_invMassB + m_invIB * ruB * ruB;

            float mass = mA + m_ratio * m_ratio * mB;

            if (mass > 0.0f)
            {
                mass = 1.0f / mass;
            }

            float C           = m_constant - lengthA - m_ratio * lengthB;
            float linearError = Math.Abs(C);

            float impulse = -mass * C;

            Vector2 PA = -impulse * uA;
            Vector2 PB = -m_ratio * impulse * uB;

            cA += m_invMassA * PA;
            aA += m_invIA * MathUtils.Cross(rA, PA);
            cB += m_invMassB * PB;
            aB += m_invIB * MathUtils.Cross(rB, PB);

            data.Positions[m_indexA].Center = cA;
            data.Positions[m_indexA].Angle = aA;
            data.Positions[m_indexB].Center = cB;
            data.Positions[m_indexB].Angle = aB;

            return linearError < Settings.LinearSlop;
        }

        /// <inheritdoc />
        public override void Dump()
        {
            int indexA = BodyA._islandIndex;
            int indexB = BodyB._islandIndex;

            Logger.Log("  b2PulleyJointDef jd;");
            Logger.Log($"  jd.bodyA = bodies[{indexA}];");
            Logger.Log($"  jd.bodyB = bodies[{indexB}];");
            Logger.Log($"  jd.collideConnected = bool({CollideConnected});");
            Logger.Log($"  jd.groundAnchorA.Set({m_groundAnchorA.X}, {m_groundAnchorA.Y});");
            Logger.Log($"  jd.groundAnchorB.Set({m_groundAnchorB.X}, {m_groundAnchorB.Y});");
            Logger.Log($"  jd.localAnchorA.Set({m_localAnchorA.X}, {m_localAnchorA.Y});");
            Logger.Log($"  jd.localAnchorB.Set({m_localAnchorB.X}, {m_localAnchorB.Y});");
            Logger.Log($"  jd.lengthA = {m_lengthA};");
            Logger.Log($"  jd.lengthB = {m_lengthB};");
            Logger.Log($"  jd.ratio = {m_ratio};");
            Logger.Log($"  joints[{Index}] = m_world.CreateJoint(&jd);");
        }

        /// <inheritdoc />
        public override void ShiftOrigin(in Vector2 newOrigin)
        {
            m_groundAnchorA -= newOrigin;
            m_groundAnchorB -= newOrigin;
        }
    };
}