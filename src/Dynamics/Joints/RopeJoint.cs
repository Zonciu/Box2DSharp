using System;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// A rope joint enforces a maximum distance between two points
    /// on two bodies. It has no other effect.
    /// Warning: if you attempt to change the maximum length during
    /// the simulation you will get some non-physical behavior.
    /// A model that would allow you to dynamically modify the length
    /// would have some sponginess, so I chose not to implement it
    /// that way. See b2DistanceJoint if you want to dynamically
    /// control length.
    public class RopeJoint : Joint
    {
        /// The local anchor point relative to bodyA's origin.
        public ref readonly Vector2 GetLocalAnchorA()
        {
            return ref m_localAnchorA;
        }

        /// The local anchor point relative to bodyB's origin.
        public ref readonly Vector2 GetLocalAnchorB()
        {
            return ref m_localAnchorB;
        }

        /// Set/Get the maximum length of the rope.
        public void SetMaxLength(float length)
        {
            m_maxLength = length;
        }

        public float GetMaxLength()
        {
            return m_maxLength;
        }

        public LimitState GetLimitState()
        {
            return m_state;
        }

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
            Vector2 F = (inv_dt * m_impulse) * m_u;
            return F;
        }

        /// <inheritdoc />
        public override float GetReactionTorque(float inv_dt)
        {
            return 0.0f;
        }

        /// Dump joint to dmLog
        public override void Dump()
        {
            int indexA = BodyA._islandIndex;
            int indexB = BodyB._islandIndex;

            Logger.Log("  b2RopeJointDef jd;");
            Logger.Log($"  jd.bodyA = bodies[{indexA}];");
            Logger.Log($"  jd.bodyB = bodies[{indexB}];");
            Logger.Log($"  jd.collideConnected = bool({CollideConnected});");
            Logger.Log($"  jd.localAnchorA.Set({m_localAnchorA.X}, {m_localAnchorA.Y});");
            Logger.Log($"  jd.localAnchorB.Set({m_localAnchorB.X}, {m_localAnchorB.Y});");
            Logger.Log($"  jd.maxLength = {m_maxLength};");
            Logger.Log($"  joints[{Index}] = m_world.CreateJoint(&jd);");
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
            m_u  = cB + m_rB - cA - m_rA;

            m_length = m_u.Length();

            float C = m_length - m_maxLength;
            if (C > 0.0f)
            {
                m_state = LimitState.AtUpperLimit;
            }
            else
            {
                m_state = LimitState.InactiveLimit;
            }

            if (m_length > Settings.LinearSlop)
            {
                m_u *= 1.0f / m_length;
            }
            else
            {
                m_u.SetZero();
                m_mass    = 0.0f;
                m_impulse = 0.0f;
                return;
            }

            // Compute effective mass.
            float crA     = MathUtils.Cross(m_rA, m_u);
            float crB     = MathUtils.Cross(m_rB, m_u);
            float invMass = m_invMassA + m_invIA * crA * crA + m_invMassB + m_invIB * crB * crB;

            m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

            if (data.Step.warmStarting)
            {
                // Scale the impulse to support a variable time step.
                m_impulse *= data.Step.dtRatio;

                Vector2 P = m_impulse * m_u;
                vA -= m_invMassA * P;
                wA -= m_invIA * MathUtils.Cross(m_rA, P);
                vB += m_invMassB * P;
                wB += m_invIB * MathUtils.Cross(m_rB, P);
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

            // Cdot = dot(u, v + cross(w, r))
            Vector2 vpA  = vA + MathUtils.Cross(wA, m_rA);
            Vector2 vpB  = vB + MathUtils.Cross(wB, m_rB);
            float   C    = m_length - m_maxLength;
            float   Cdot = MathUtils.Dot(m_u, vpB - vpA);

            // Predictive constraint.
            if (C < 0.0f)
            {
                Cdot += data.Step.inv_dt * C;
            }

            float impulse    = -m_mass * Cdot;
            float oldImpulse = m_impulse;
            m_impulse = Math.Min(0.0f, m_impulse + impulse);
            impulse   = m_impulse - oldImpulse;

            Vector2 P = impulse * m_u;
            vA -= m_invMassA * P;
            wA -= m_invIA * MathUtils.Cross(m_rA, P);
            vB += m_invMassB * P;
            wB += m_invIB * MathUtils.Cross(m_rB, P);

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
            Vector2 u  = cB + rB - cA - rA;

            float length = u.Normalize();
            float C      = length - m_maxLength;

            C = MathUtils.Clamp(C, 0.0f, Settings.MaxLinearCorrection);

            float   impulse = -m_mass * C;
            Vector2 P       = impulse * u;

            cA -= m_invMassA * P;
            aA -= m_invIA * MathUtils.Cross(rA, P);
            cB += m_invMassB * P;
            aB += m_invIB * MathUtils.Cross(rB, P);

            data.Positions[m_indexA].Center = cA;
            data.Positions[m_indexA].Angle = aA;
            data.Positions[m_indexB].Center = cB;
            data.Positions[m_indexB].Angle = aB;

            return length - m_maxLength < Settings.LinearSlop;
        }

        internal RopeJoint(RopeJointDef def) : base(def)
        {
            m_localAnchorA = def.localAnchorA;
            m_localAnchorB = def.localAnchorB;

            m_maxLength = def.maxLength;

            m_mass    = 0.0f;
            m_impulse = 0.0f;
            m_state   = LimitState.InactiveLimit;
            m_length  = 0.0f;
        }

        // Solver shared
        internal Vector2 m_localAnchorA;

        internal Vector2 m_localAnchorB;

        internal float m_maxLength;

        internal float m_length;

        internal float m_impulse;

        // Solver temp
        internal int m_indexA;

        internal int m_indexB;

        internal Vector2 m_u;

        internal Vector2 m_rA;

        internal Vector2 m_rB;

        internal Vector2 m_localCenterA;

        internal Vector2 m_localCenterB;

        internal float m_invMassA;

        internal float m_invMassB;

        internal float m_invIA;

        internal float m_invIB;

        internal float m_mass;

        internal LimitState m_state;
    };
}