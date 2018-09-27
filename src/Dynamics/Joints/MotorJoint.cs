using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// A motor joint is used to control the relative motion
    /// between two bodies. A typical usage is to control the movement
    /// of a dynamic body with respect to the ground.
    public class MotorJoint : Joint
    {
        /// Set/get the target linear offset, in frame A, in meters.
        void SetLinearOffset(in Vector2 linearOffset)
        {
            if (linearOffset.X != m_linearOffset.X || linearOffset.Y != m_linearOffset.Y)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                m_linearOffset  = linearOffset;
            }
        }

        ref readonly Vector2 GetLinearOffset()
        {
            return ref m_linearOffset;
        }

        /// Set/get the target angular offset, in radians.
        void SetAngularOffset(float angularOffset)
        {
            if (angularOffset != m_angularOffset)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                m_angularOffset = angularOffset;
            }
        }

        float GetAngularOffset()
        {
            return m_angularOffset;
        }

        /// Set the maximum friction force in N.
        void SetMaxForce(float force)
        {
            Debug.Assert(force.IsValid() && force >= 0.0f);
            m_maxForce = force;
        }

        /// Get the maximum friction force in N.
        float GetMaxForce()
        {
            return m_maxForce;
        }

        /// Set the maximum friction torque in N*m.
        void SetMaxTorque(float torque)
        {
            Debug.Assert(torque.IsValid() && torque >= 0.0f);
            m_maxTorque = torque;
        }

        /// Get the maximum friction torque in N*m.
        float GetMaxTorque()
        {
            return m_maxTorque;
        }

        /// Set the position correction factor in the range [0,1].
        void SetCorrectionFactor(float factor)
        {
            Debug.Assert(factor.IsValid() && 0.0f <= factor && factor <= 1.0f);
            m_correctionFactor = factor;
        }

        /// Get the position correction factor in the range [0,1].
        float GetCorrectionFactor()
        {
            return m_correctionFactor;
        }

        /// <inheritdoc />
        public override Vector2 GetAnchorA()
        {
            return BodyA.GetPosition();
        }

        /// <inheritdoc />
        public override Vector2 GetAnchorB()
        {
            return BodyB.GetPosition();
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

        /// Dump to Logger.Log
        public override void Dump()
        {
            int indexA = BodyA._islandIndex;
            int indexB = BodyB._islandIndex;

            Logger.Log("  b2MotorJointDef jd;");
            Logger.Log($"  jd.bodyA = bodies[{indexA}];");
            Logger.Log($"  jd.bodyB = bodies[{indexB}];");
            Logger.Log($"  jd.collideConnected = bool({CollideConnected});");
            Logger.Log($"  jd.linearOffset.Set({m_linearOffset.X}, {m_linearOffset.Y});");
            Logger.Log($"  jd.angularOffset = {m_angularOffset};");
            Logger.Log($"  jd.maxForce = {m_maxForce};");
            Logger.Log($"  jd.maxTorque = {m_maxTorque};");
            Logger.Log($"  jd.correctionFactor = {m_correctionFactor};");
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

            // Compute the effective mass matrix.
            m_rA = MathUtils.Mul(qA, m_linearOffset - m_localCenterA);
            m_rB = MathUtils.Mul(qB, -m_localCenterB);

            // J = [-I -r1_skew I r2_skew]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
            //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
            //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA,    iB = m_invIB;

            // Upper 2 by 2 of K for point to point
            Matrix2x2 K = new Matrix2x2();
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

            m_linearError  = cB + m_rB - cA - m_rA;
            m_angularError = aB - aA - m_angularOffset;

            if (data.Step.warmStarting)
            {
                // Scale impulses to support a variable time step.
                m_linearImpulse  *= data.Step.dtRatio;
                m_angularImpulse *= data.Step.dtRatio;

                Vector2 P = new Vector2(m_linearImpulse.X, m_linearImpulse.Y);
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

            float h     = data.Step.dt;
            float inv_h = data.Step.inv_dt;

            // Solve angular friction
            {
                float Cdot    = wB - wA + inv_h * m_correctionFactor * m_angularError;
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
                Vector2 Cdot = vB
                             + MathUtils.Cross(wB, m_rB)
                             - vA
                             - MathUtils.Cross(wA, m_rA)
                             + inv_h * m_correctionFactor * m_linearError;

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

        internal MotorJoint(MotorJointDef def) : base(def)
        {
            m_linearOffset  = def.linearOffset;
            m_angularOffset = def.angularOffset;

            m_linearImpulse.SetZero();
            m_angularImpulse = 0.0f;

            m_maxForce         = def.maxForce;
            m_maxTorque        = def.maxTorque;
            m_correctionFactor = def.correctionFactor;
        }

        // Solver shared
        Vector2 m_linearOffset;

        float m_angularOffset;

        Vector2 m_linearImpulse;

        float m_angularImpulse;

        float m_maxForce;

        float m_maxTorque;

        float m_correctionFactor;

        // Solver temp
        int m_indexA;

        int m_indexB;

        Vector2 m_rA;

        Vector2 m_rB;

        Vector2 m_localCenterA;

        Vector2 m_localCenterB;

        Vector2 m_linearError;

        float m_angularError;

        float m_invMassA;

        float m_invMassB;

        float m_invIA;

        float m_invIB;

        Matrix2x2 m_linearMass;

        float m_angularMass;
    };

    
}