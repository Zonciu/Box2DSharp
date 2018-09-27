using System;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// A distance joint constrains two points on two bodies
    /// to remain at a fixed distance from each other. You can view
    /// this as a massless, rigid rod.
    public class DistanceJoint : Joint
    {
        private float m_bias;

        private float m_gamma;

        private float m_impulse;

        // Solver temp
        private int m_indexA;

        private int m_indexB;

        private float m_invIA;

        private float m_invIB;

        private float m_invMassA;

        private float m_invMassB;

        // Solver shared
        private readonly Vector2 m_localAnchorA;

        private readonly Vector2 m_localAnchorB;

        private Vector2 m_localCenterA;

        private Vector2 m_localCenterB;

        private float m_mass;

        private Vector2 m_rA;

        private Vector2 m_rB;

        private Vector2 m_u;

        internal DistanceJoint(DistanceJointDef def) : base(def)
        {
            m_localAnchorA = def.localAnchorA;
            m_localAnchorB = def.localAnchorB;
            Length         = def.length;
            FrequencyHz    = def.frequencyHz;
            DampingRatio   = def.dampingRatio;
            m_impulse      = 0.0f;
            m_gamma        = 0.0f;
            m_bias         = 0.0f;
        }

        /// Set/get the natural length.
        /// Manipulating the length can lead to non-physical behavior when the frequency is zero.
        public float Length { get; set; }

        /// Set/get frequency in Hz.
        public float FrequencyHz { get; set; }

        /// Set/get damping ratio.

        public float DampingRatio { get; set; }

        public override Vector2 GetAnchorA()
        {
            return BodyA.GetWorldPoint(m_localAnchorA);
        }

        public override Vector2 GetAnchorB()
        {
            return BodyB.GetWorldPoint(m_localAnchorB);
        }

        /// Get the reaction force given the inverse time step.
        /// Unit is N.
        public override Vector2 GetReactionForce(float inv_dt)
        {
            var F = inv_dt * m_impulse * m_u;
            return F;
        }

        /// Get the reaction torque given the inverse time step.
        /// Unit is N*m. This is always zero for a distance joint.
        public override float GetReactionTorque(float inv_dt)
        {
            return 0.0f;
        }

        /// The local anchor point relative to bodyA's origin.
        private ref readonly Vector2 GetLocalAnchorA()
        {
            return ref m_localAnchorA;
        }

        /// The local anchor point relative to bodyB's origin.
        private ref readonly Vector2 GetLocalAnchorB()
        {
            return ref m_localAnchorB;
        }

        /// Dump joint to dmLog
        public override void Dump()
        {
            var indexA = BodyA._islandIndex;
            var indexB = BodyB._islandIndex;

            Logger.Log("  b2DistanceJointDef jd;");
            Logger.Log($"  jd.bodyA = bodies[{indexA}];");
            Logger.Log($"  jd.bodyB = bodies[{indexB}];");
            Logger.Log($"  jd.collideConnected = bool({CollideConnected});");
            Logger.Log($"  jd.localAnchorA.Set({m_localAnchorA.X}, {m_localAnchorA.Y});");
            Logger.Log($"  jd.localAnchorB.Set({m_localAnchorB.X}, {m_localAnchorB.Y});");
            Logger.Log($"  jd.length = {Length};");
            Logger.Log($"  jd.frequencyHz = {FrequencyHz};");
            Logger.Log($"  jd.dampingRatio = {DampingRatio};");
            Logger.Log($"  joints[{Index}] = m_world.CreateJoint(&jd);");
        }

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

            var cA = data.Positions[m_indexA].Center;
            var aA = data.Positions[m_indexA].Angle;
            var vA = data.Velocities[m_indexA].v;
            var wA = data.Velocities[m_indexA].w;

            var cB = data.Positions[m_indexB].Center;
            var aB = data.Positions[m_indexB].Angle;
            var vB = data.Velocities[m_indexB].v;
            var wB = data.Velocities[m_indexB].w;

            var qA = new Rotation(aA);
            var qB = new Rotation(aB);

            m_rA = MathUtils.Mul(qA, m_localAnchorA - m_localCenterA);
            m_rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);
            m_u  = cB + m_rB - cA - m_rA;

            // Handle singularity.
            var length = m_u.Length();
            if (length > Settings.LinearSlop)
            {
                m_u *= 1.0f / length;
            }
            else
            {
                m_u.Set(0.0f, 0.0f);
            }

            var crAu    = MathUtils.Cross(m_rA, m_u);
            var crBu    = MathUtils.Cross(m_rB, m_u);
            var invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu;

            // Compute the effective mass matrix.
            m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

            if (FrequencyHz > 0.0f)
            {
                var C = length - length;

                // Frequency
                var omega = 2.0f * Settings.Pi * FrequencyHz;

                // Damping coefficient
                var d = 2.0f * m_mass * DampingRatio * omega;

                // Spring stiffness
                var k = m_mass * omega * omega;

                // magic formulas
                var h = data.Step.dt;
                m_gamma = h * (d + h * k);
                m_gamma = !m_gamma.Equals(0.0f) ? 1.0f / m_gamma : 0.0f;
                m_bias  = C * h * k * m_gamma;

                invMass += m_gamma;
                m_mass  =  !invMass.Equals(0.0f) ? 1.0f / invMass : 0.0f;
            }
            else
            {
                m_gamma = 0.0f;
                m_bias  = 0.0f;
            }

            if (data.Step.warmStarting)
            {
                // Scale the impulse to support a variable time step.
                m_impulse *= data.Step.dtRatio;

                var P = m_impulse * m_u;
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

        internal override void SolveVelocityConstraints(SolverData data)
        {
            var vA = data.Velocities[m_indexA].v;
            var wA = data.Velocities[m_indexA].w;
            var vB = data.Velocities[m_indexB].v;
            var wB = data.Velocities[m_indexB].w;

            // Cdot = dot(u, v + cross(w, r))
            var vpA  = vA + MathUtils.Cross(wA, m_rA);
            var vpB  = vB + MathUtils.Cross(wB, m_rB);
            var Cdot = MathUtils.Dot(m_u, vpB - vpA);

            var impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
            m_impulse += impulse;

            var P = impulse * m_u;
            vA -= m_invMassA * P;
            wA -= m_invIA * MathUtils.Cross(m_rA, P);
            vB += m_invMassB * P;
            wB += m_invIB * MathUtils.Cross(m_rB, P);

            data.Velocities[m_indexA].v = vA;
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v = vB;
            data.Velocities[m_indexB].w = wB;
        }

        internal override bool SolvePositionConstraints(SolverData data)
        {
            if (FrequencyHz > 0.0f)
            {
                // There is no position correction for soft distance constraints.
                return true;
            }

            var cA = data.Positions[m_indexA].Center;
            var aA = data.Positions[m_indexA].Angle;
            var cB = data.Positions[m_indexB].Center;
            var aB = data.Positions[m_indexB].Angle;

            var qA = new Rotation(aA);
            var qB = new Rotation(aB);

            var rA = MathUtils.Mul(qA, m_localAnchorA - m_localCenterA);
            var rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);
            var u  = cB + rB - cA - rA;

            var length = u.Normalize();
            var C      = length - Length;
            C = MathUtils.Clamp(C, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);

            var impulse = -m_mass * C;
            var P       = impulse * u;

            cA -= m_invMassA * P;
            aA -= m_invIA * MathUtils.Cross(rA, P);
            cB += m_invMassB * P;
            aB += m_invIB * MathUtils.Cross(rB, P);

            data.Positions[m_indexA].Center = cA;
            data.Positions[m_indexA].Angle = aA;
            data.Positions[m_indexB].Center = cB;
            data.Positions[m_indexB].Angle = aB;

            return Math.Abs(C) < Settings.LinearSlop;
        }
    }
}