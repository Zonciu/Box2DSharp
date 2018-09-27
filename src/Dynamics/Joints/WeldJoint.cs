using System;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// A weld joint essentially glues two bodies together. A weld joint may
    /// distort somewhat because the island constraint solver is approximate.
    public class WeldJoint : Joint
    {
        /// The local anchor point relative to bodyA's origin.
        ref readonly Vector2 GetLocalAnchorA()
        {
            return ref m_localAnchorA;
        }

        /// The local anchor point relative to bodyB's origin.
        ref readonly Vector2 GetLocalAnchorB()
        {
            return ref m_localAnchorB;
        }

        /// Get the reference angle.
        float GetReferenceAngle()
        {
            return m_referenceAngle;
        }

        /// Set/get frequency in Hz.
        void SetFrequency(float hz)
        {
            m_frequencyHz = hz;
        }

        float GetFrequency()
        {
            return m_frequencyHz;
        }

        /// Set/get damping ratio.
        void SetDampingRatio(float ratio)
        {
            m_dampingRatio = ratio;
        }

        float GetDampingRatio()
        {
            return m_dampingRatio;
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
            Vector2 P = new Vector2(m_impulse.X, m_impulse.Y);
            return inv_dt * P;
        }

        /// <inheritdoc />
        public override float GetReactionTorque(float inv_dt)
        {
            return inv_dt * m_impulse.Z;
        }

        /// Dump to Logger.Log
        public override void Dump()
        {
            int indexA = BodyA._islandIndex;
            int indexB = BodyB._islandIndex;

            Logger.Log("  b2WeldJointDef jd;");
            Logger.Log($"  jd.bodyA = bodies[{indexA}];");
            Logger.Log($"  jd.bodyB = bodies[{indexB}];");
            Logger.Log($"  jd.collideConnected = bool({CollideConnected});");
            Logger.Log($"  jd.localAnchorA.Set({m_localAnchorA.X}, {m_localAnchorA.Y});");
            Logger.Log($"  jd.localAnchorB.Set({m_localAnchorB.X}, {m_localAnchorB.Y});");
            Logger.Log($"  jd.referenceAngle = {m_referenceAngle};");
            Logger.Log($"  jd.frequencyHz = {m_frequencyHz};");
            Logger.Log($"  jd.dampingRatio = {m_dampingRatio};");
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

            float   aA = data.Positions[m_indexA].Angle;
            Vector2 vA = data.Velocities[m_indexA].v;
            float   wA = data.Velocities[m_indexA].w;

            float   aB = data.Positions[m_indexB].Angle;
            Vector2 vB = data.Velocities[m_indexB].v;
            float   wB = data.Velocities[m_indexB].w;

            var qA = new Rotation(aA);
            var qB = new Rotation(aB);

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

            Matrix3x3 K = new Matrix3x3();
            K.ex.X = mA + mB + m_rA.Y * m_rA.Y * iA + m_rB.Y * m_rB.Y * iB;
            K.ey.X = -m_rA.Y * m_rA.X * iA - m_rB.Y * m_rB.X * iB;
            K.ez.X = -m_rA.Y * iA - m_rB.Y * iB;
            K.ex.Y = K.ey.X;
            K.ey.Y = mA + mB + m_rA.X * m_rA.X * iA + m_rB.X * m_rB.X * iB;
            K.ez.Y = m_rA.X * iA + m_rB.X * iB;
            K.ex.Z = K.ez.X;
            K.ey.Z = K.ez.Y;
            K.ez.Z = iA + iB;

            if (m_frequencyHz > 0.0f)
            {
                K.GetInverse22(ref m_mass);

                float invM = iA + iB;
                float m    = invM > 0.0f ? 1.0f / invM : 0.0f;

                float C = aB - aA - m_referenceAngle;

                // Frequency
                float omega = 2.0f * Settings.Pi * m_frequencyHz;

                // Damping coefficient
                float d = 2.0f * m * m_dampingRatio * omega;

                // Spring stiffness
                float k = m * omega * omega;

                // magic formulas
                float h = data.Step.dt;
                m_gamma = h * (d + h * k);
                m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
                m_bias  = C * h * k * m_gamma;

                invM        += m_gamma;
                m_mass.ez.Z =  invM != 0.0f ? 1.0f / invM : 0.0f;
            }
            else if (K.ez.Z == 0.0f)
            {
                K.GetInverse22(ref m_mass);
                m_gamma = 0.0f;
                m_bias  = 0.0f;
            }
            else
            {
                K.GetSymInverse33(ref m_mass);
                m_gamma = 0.0f;
                m_bias  = 0.0f;
            }

            if (data.Step.warmStarting)
            {
                // Scale impulses to support a variable time step.
                m_impulse *= data.Step.dtRatio;

                Vector2 P = new Vector2(m_impulse.X, m_impulse.Y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(m_rA, P) + m_impulse.Z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(m_rB, P) + m_impulse.Z);
            }
            else
            {
                m_impulse.SetZero();
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

            if (m_frequencyHz > 0.0f)
            {
                float Cdot2 = wB - wA;

                float impulse2 = -m_mass.ez.Z * (Cdot2 + m_bias + m_gamma * m_impulse.Z);
                m_impulse.Z += impulse2;

                wA -= iA * impulse2;
                wB += iB * impulse2;

                Vector2 Cdot1 = vB + MathUtils.Cross(wB, m_rB) - vA - MathUtils.Cross(wA, m_rA);

                Vector2 impulse1 = -MathUtils.Mul22(m_mass, Cdot1);
                m_impulse.X += impulse1.X;
                m_impulse.Y += impulse1.Y;

                Vector2 P = impulse1;

                vA -= mA * P;
                wA -= iA * MathUtils.Cross(m_rA, P);

                vB += mB * P;
                wB += iB * MathUtils.Cross(m_rB, P);
            }
            else
            {
                Vector2 Cdot1 = vB + MathUtils.Cross(wB, m_rB) - vA - MathUtils.Cross(wA, m_rA);
                float   Cdot2 = wB - wA;
                Vector3 Cdot  = new Vector3(Cdot1.X, Cdot1.Y, Cdot2);

                Vector3 impulse = -MathUtils.Mul(m_mass, Cdot);
                m_impulse += impulse;

                Vector2 P = new Vector2(impulse.X, impulse.Y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(m_rA, P) + impulse.Z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(m_rB, P) + impulse.Z);
            }

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

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA,    iB = m_invIB;

            Vector2 rA = MathUtils.Mul(qA, m_localAnchorA - m_localCenterA);
            Vector2 rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);

            float positionError, angularError;

            Matrix3x3 K = new Matrix3x3();
            K.ex.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
            K.ey.X = -rA.Y * rA.X * iA - rB.Y * rB.X * iB;
            K.ez.X = -rA.Y * iA - rB.Y * iB;
            K.ex.Y = K.ey.X;
            K.ey.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
            K.ez.Y = rA.X * iA + rB.X * iB;
            K.ex.Z = K.ez.X;
            K.ey.Z = K.ez.Y;
            K.ez.Z = iA + iB;

            if (m_frequencyHz > 0.0f)
            {
                Vector2 C1 = cB + rB - cA - rA;

                positionError = C1.Length();
                angularError  = 0.0f;

                Vector2 P = -K.Solve22(C1);

                cA -= mA * P;
                aA -= iA * MathUtils.Cross(rA, P);

                cB += mB * P;
                aB += iB * MathUtils.Cross(rB, P);
            }
            else
            {
                Vector2 C1 = cB + rB - cA - rA;
                float   C2 = aB - aA - m_referenceAngle;

                positionError = C1.Length();
                angularError  = Math.Abs(C2);

                Vector3 C = new Vector3(C1.X, C1.Y, C2);

                Vector3 impulse = new Vector3();
                if (K.ez.Z > 0.0f)
                {
                    impulse = -K.Solve33(C);
                }
                else
                {
                    Vector2 impulse2 = -K.Solve22(C1);
                    impulse.Set(impulse2.X, impulse2.Y, 0.0f);
                }

                Vector2 P = new Vector2(impulse.X, impulse.Y);

                cA -= mA * P;
                aA -= iA * (MathUtils.Cross(rA, P) + impulse.Z);

                cB += mB * P;
                aB += iB * (MathUtils.Cross(rB, P) + impulse.Z);
            }

            data.Positions[m_indexA].Center = cA;
            data.Positions[m_indexA].Angle = aA;
            data.Positions[m_indexB].Center = cB;
            data.Positions[m_indexB].Angle = aB;

            return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }

        internal WeldJoint(WeldJointDef def) : base(def)
        {
            m_localAnchorA   = def.localAnchorA;
            m_localAnchorB   = def.localAnchorB;
            m_referenceAngle = def.referenceAngle;
            m_frequencyHz    = def.frequencyHz;
            m_dampingRatio   = def.dampingRatio;

            m_impulse.SetZero();
        }

        float m_frequencyHz;

        float m_dampingRatio;

        float m_bias;

        // Solver shared
        Vector2 m_localAnchorA;

        Vector2 m_localAnchorB;

        float m_referenceAngle;

        float m_gamma;

        Vector3 m_impulse;

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

        Matrix3x3 m_mass;
    };
}