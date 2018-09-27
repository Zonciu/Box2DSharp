using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// Friction joint. This is used for top-down friction.
    /// It provides 2D translational friction and angular friction.
    public class FrictionJoint : Joint
    {
        /// The local anchor point relative to bodyA's origin.
        public ref Vector2 GetLocalAnchorA()
        {
            return ref m_localAnchorA;
        }

        /// The local anchor point relative to bodyB's origin.
        public ref Vector2 GetLocalAnchorB()
        {
            return ref m_localAnchorB;
        }

        /// Get/Set the maximum friction force in N.

        public float MaxForce
        {
            get => m_maxForce;
            set
            {
                Debug.Assert(value.IsValid() && value >= 0.0f);
                m_maxForce = value;
            }
        }

        public float MaxTorque
        {
            get => m_maxTorque;
            set
            {
                Debug.Assert(value.IsValid() && value >= 0.0f);
                m_maxTorque = value;
            }
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
            return inv_dt * m_linearImpulse;
        }

        /// <inheritdoc />
        public override float GetReactionTorque(float inv_dt)
        {
            return inv_dt * m_angularImpulse;
        }

        /// Dump joint to dmLog
        public override void Dump()
        { }

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

            float   aA = data.Positions[m_indexA].Angle;
            Vector2 vA = data.Velocities[m_indexA].v;
            float   wA = data.Velocities[m_indexA].w;

            float   aB = data.Positions[m_indexB].Angle;
            Vector2 vB = data.Velocities[m_indexB].v;
            float   wB = data.Velocities[m_indexB].w;

            var qA = new Rotation(aA);
            var qB = new Rotation(aB);

            // Compute the effective mass matrix.
            m_rA = MathUtils.Mul(qA, m_localAnchorA - m_localCenterA);
            m_rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);

            // J = [-I -r1_skew I r2_skew]
            //     [ 0       -1 0       1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
            //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
            //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA,    iB = m_invIB;

            var K = new Matrix2x2();
            K.ex.X = mA + mB + iA * m_rA.Y * m_rA.Y + iB * m_rB.Y * m_rB.Y;
            K.ex.Y = -iA * m_rA.X * m_rA.Y - iB * m_rB.X * m_rB.Y;
            K.ey.X = K.ex.Y;
            K.ey.Y = mA + mB + iA * m_rA.X * m_rA.X + iB * m_rB.X * m_rB.X;

            m_linearMass = K.GetInverse();

            m_angularMass = iA + iB;
            if (m_angularMass > 0.0f)
            {
                m_angularMass = 1.0f / m_angularMass;
            }

            if (data.Step.warmStarting)
            {
                // Scale impulses to support a variable time step.
                m_linearImpulse  *= data.Step.dtRatio;
                m_angularImpulse *= data.Step.dtRatio;

                var P = new Vector2(m_linearImpulse.X, m_linearImpulse.Y);
                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(m_rA, P) + m_angularImpulse);
                vB += mB * P;
                wB += iB * (MathUtils.Cross(m_rB, P) + m_angularImpulse);
            }
            else
            {
                m_linearImpulse.SetZero();
                m_angularImpulse = 0.0f;
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

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA,    iB = m_invIB;

            float h = data.Step.dt;

            // Solve angular friction
            {
                float Cdot    = wB - wA;
                float impulse = -m_angularMass * Cdot;

                float oldImpulse = m_angularImpulse;
                float maxImpulse = h * m_maxTorque;
                m_angularImpulse = MathUtils.Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
                impulse          = m_angularImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve linear friction
            {
                Vector2 Cdot = vB + MathUtils.Cross(wB, m_rB) - vA - MathUtils.Cross(wA, m_rA);

                Vector2 impulse    = -MathUtils.Mul(m_linearMass, Cdot);
                Vector2 oldImpulse = m_linearImpulse;
                m_linearImpulse += impulse;

                float maxImpulse = h * m_maxForce;

                if (m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
                {
                    m_linearImpulse.Normalize();
                    m_linearImpulse *= maxImpulse;
                }

                impulse = m_linearImpulse - oldImpulse;

                vA -= mA * impulse;
                wA -= iA * MathUtils.Cross(m_rA, impulse);

                vB += mB * impulse;
                wB += iB * MathUtils.Cross(m_rB, impulse);
            }

            data.Velocities[m_indexA].v = vA;
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v = vB;
            data.Velocities[m_indexB].w = wB;
        }

        /// <inheritdoc />
        internal override bool SolvePositionConstraints(SolverData data)
        {
            return true;
        }

        internal FrictionJoint(FrictionJointDef def) : base(def)
        {
            m_localAnchorA = def.localAnchorA;
            m_localAnchorB = def.localAnchorB;

            m_linearImpulse.SetZero();
            m_angularImpulse = 0.0f;

            m_maxForce  = def.maxForce;
            m_maxTorque = def.maxTorque;
        }

        Vector2 m_localAnchorA;

        Vector2 m_localAnchorB;

        // Solver shared
        Vector2 m_linearImpulse;

        float m_angularImpulse;

        float m_maxForce;

        float m_maxTorque;

        // Solver temp
        int m_indexA;

        int m_indexB;

        Vector2 m_rA;

        Vector2 m_rB;

        Vector2 m_localCenterA;

        Vector2 m_localCenterB;

        float m_invMassA;

        float m_invMassB;

        float m_invIA;

        float m_invIB;

        Matrix2x2 m_linearMass;

        float m_angularMass;
    };
}