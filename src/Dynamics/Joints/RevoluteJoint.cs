using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// A revolute joint constrains two bodies to share a common point while they
    /// are free to rotate about the point. The relative rotation about the shared
    /// point is the joint angle. You can limit the relative rotation with
    /// a joint limit that specifies a lower and upper angle. You can use a motor
    /// to drive the relative rotation about the shared point. A maximum motor torque
    /// is provided so that infinite forces are not generated.
    internal class RevoluteJoint : Joint
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

        /// Get the reference angle.
        public float GetReferenceAngle()
        {
            return m_referenceAngle;
        }

        /// Get the current joint angle in radians.
        public float GetJointAngle()
        {
            var bA = BodyA;
            var bB = BodyB;
            return bB._sweep.a - bA._sweep.a - m_referenceAngle;
        }

        /// Get the current joint angle speed in radians per second.
        public float GetJointSpeed()
        {
            var bA = BodyA;
            var bB = BodyB;
            return bB._angularVelocity - bA._angularVelocity;
        }

        /// Is the joint limit enabled?
        public bool IsLimitEnabled()
        {
            return m_enableLimit;
        }

        /// Enable/disable the joint limit.
        public void EnableLimit(bool flag)
        {
            if (flag != m_enableLimit)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                m_enableLimit   = flag;
                m_impulse.Z     = 0.0f;
            }
        }

        /// Get the lower joint limit in radians.
        public float GetLowerLimit()
        {
            return m_lowerAngle;
        }

        /// Get the upper joint limit in radians.
        public float GetUpperLimit()
        {
            return m_upperAngle;
        }

        /// Set the joint limits in radians.
        public void SetLimits(float lower, float upper)
        {
            Debug.Assert(lower <= upper);

            if (lower != m_lowerAngle || upper != m_upperAngle)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                m_impulse.Z     = 0.0f;
                m_lowerAngle    = lower;
                m_upperAngle    = upper;
            }
        }

        /// Is the joint motor enabled?
        public bool IsMotorEnabled()
        {
            return m_enableMotor;
        }

        /// Enable/disable the joint motor.
        public void EnableMotor(bool flag)
        {
            if (flag != m_enableMotor)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                m_enableMotor   = flag;
            }
        }

        /// Set the motor speed in radians per second.
        public void SetMotorSpeed(float speed)
        {
            if (speed != m_motorSpeed)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                m_motorSpeed    = speed;
            }
        }

        /// Get the motor speed in radians per second.
        public float GetMotorSpeed()
        {
            return m_motorSpeed;
        }

        /// Set the maximum motor torque, usually in N-m.
        public void SetMaxMotorTorque(float torque)
        {
            if (torque != m_maxMotorTorque)
            {
                BodyA.IsAwake  = true;
                BodyB.IsAwake  = true;
                m_maxMotorTorque = torque;
            }
        }

        public float GetMaxMotorTorque()
        {
            return m_maxMotorTorque;
        }

        /// Get the reaction force given the inverse time step.
        /// Unit is N.
        /// Get the current motor torque given the inverse time step.
        /// Unit is N*m.
        public float GetMotorTorque(float inv_dt)
        {
            return inv_dt * m_motorImpulse;
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

        /// Dump to Logger.Log.
        public override void Dump()
        {
            int indexA = BodyA._islandIndex;
            int indexB = BodyB._islandIndex;

            Logger.Log("  b2RevoluteJointDef jd;");
            Logger.Log($"  jd.bodyA = bodies[{indexA}];");
            Logger.Log($"  jd.bodyB = bodies[{indexB}];");
            Logger.Log($"  jd.collideConnected = bool({CollideConnected});");
            Logger.Log($"  jd.localAnchorA.Set({m_localAnchorA.X}, {m_localAnchorA.Y});");
            Logger.Log($"  jd.localAnchorB.Set({m_localAnchorB.X}, {m_localAnchorB.Y});");
            Logger.Log($"  jd.referenceAngle = {m_referenceAngle};");
            Logger.Log($"  jd.enableLimit = bool({m_enableLimit});");
            Logger.Log($"  jd.lowerAngle = {m_lowerAngle};");
            Logger.Log($"  jd.upperAngle = {m_upperAngle};");
            Logger.Log($"  jd.enableMotor = bool({m_enableMotor});");
            Logger.Log($"  jd.motorSpeed = {m_motorSpeed};");
            Logger.Log($"  jd.maxMotorTorque = {m_maxMotorTorque};");
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

            bool fixedRotation = (iA + iB == 0.0f);

            m_mass.ex.X = mA + mB + m_rA.Y * m_rA.Y * iA + m_rB.Y * m_rB.Y * iB;
            m_mass.ey.X = -m_rA.Y * m_rA.X * iA - m_rB.Y * m_rB.X * iB;
            m_mass.ez.X = -m_rA.Y * iA - m_rB.Y * iB;
            m_mass.ex.Y = m_mass.ey.X;
            m_mass.ey.Y = mA + mB + m_rA.X * m_rA.X * iA + m_rB.X * m_rB.X * iB;
            m_mass.ez.Y = m_rA.X * iA + m_rB.X * iB;
            m_mass.ex.Z = m_mass.ez.X;
            m_mass.ey.Z = m_mass.ez.Y;
            m_mass.ez.Z = iA + iB;

            m_motorMass = iA + iB;
            if (m_motorMass > 0.0f)
            {
                m_motorMass = 1.0f / m_motorMass;
            }

            if (m_enableMotor == false || fixedRotation)
            {
                m_motorImpulse = 0.0f;
            }

            if (m_enableLimit && fixedRotation == false)
            {
                float jointAngle = aB - aA - m_referenceAngle;
                if (Math.Abs(m_upperAngle - m_lowerAngle) < 2.0f * Settings.AngularSlop)
                {
                    m_limitState = LimitState.EqualLimits;
                }
                else if (jointAngle <= m_lowerAngle)
                {
                    if (m_limitState != LimitState.AtLowerLimit)
                    {
                        m_impulse.Z = 0.0f;
                    }

                    m_limitState = LimitState.AtLowerLimit;
                }
                else if (jointAngle >= m_upperAngle)
                {
                    if (m_limitState != LimitState.AtUpperLimit)
                    {
                        m_impulse.Z = 0.0f;
                    }

                    m_limitState = LimitState.AtUpperLimit;
                }
                else
                {
                    m_limitState = LimitState.InactiveLimit;
                    m_impulse.Z  = 0.0f;
                }
            }
            else
            {
                m_limitState = LimitState.InactiveLimit;
            }

            if (data.Step.warmStarting)
            {
                // Scale impulses to support a variable time step.
                m_impulse      *= data.Step.dtRatio;
                m_motorImpulse *= data.Step.dtRatio;

                Vector2 P = new Vector2(m_impulse.X, m_impulse.Y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(m_rA, P) + m_motorImpulse + m_impulse.Z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(m_rB, P) + m_motorImpulse + m_impulse.Z);
            }
            else
            {
                m_impulse.SetZero();
                m_motorImpulse = 0.0f;
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

            bool fixedRotation = (iA + iB == 0.0f);

            // Solve motor constraint.
            if (m_enableMotor && m_limitState != LimitState.EqualLimits && fixedRotation == false)
            {
                float Cdot       = wB - wA - m_motorSpeed;
                float impulse    = -m_motorMass * Cdot;
                float oldImpulse = m_motorImpulse;
                float maxImpulse = data.Step.dt * m_maxMotorTorque;
                m_motorImpulse = MathUtils.Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse        = m_motorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve limit constraint.
            if (m_enableLimit && m_limitState != LimitState.InactiveLimit && fixedRotation == false)
            {
                Vector2 Cdot1 = vB + MathUtils.Cross(wB, m_rB) - vA - MathUtils.Cross(wA, m_rA);
                float   Cdot2 = wB - wA;
                var     Cdot  = new Vector3(Cdot1.X, Cdot1.Y, Cdot2);

                var impulse = -m_mass.Solve33(Cdot);

                if (m_limitState == LimitState.EqualLimits)
                {
                    m_impulse += impulse;
                }
                else if (m_limitState == LimitState.AtLowerLimit)
                {
                    float newImpulse = m_impulse.Z + impulse.Z;
                    if (newImpulse < 0.0f)
                    {
                        var rhs     = -Cdot1 + m_impulse.Z * new Vector2(m_mass.ez.X, m_mass.ez.Y);
                        var reduced = m_mass.Solve22(rhs);
                        impulse.X   =  reduced.X;
                        impulse.Y   =  reduced.Y;
                        impulse.Z   =  -m_impulse.Z;
                        m_impulse.X += reduced.X;
                        m_impulse.Y += reduced.Y;
                        m_impulse.Z =  0.0f;
                    }
                    else
                    {
                        m_impulse += impulse;
                    }
                }
                else if (m_limitState == LimitState.AtUpperLimit)
                {
                    float newImpulse = m_impulse.Z + impulse.Z;
                    if (newImpulse > 0.0f)
                    {
                        var rhs     = -Cdot1 + m_impulse.Z * new Vector2(m_mass.ez.X, m_mass.ez.Y);
                        var reduced = m_mass.Solve22(rhs);
                        impulse.X   =  reduced.X;
                        impulse.Y   =  reduced.Y;
                        impulse.Z   =  -m_impulse.Z;
                        m_impulse.X += reduced.X;
                        m_impulse.Y += reduced.Y;
                        m_impulse.Z =  0.0f;
                    }
                    else
                    {
                        m_impulse += impulse;
                    }
                }

                Vector2 P = new Vector2(impulse.X, impulse.Y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(m_rA, P) + impulse.Z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(m_rB, P) + impulse.Z);
            }
            else
            {
                // Solve point-to-point constraint
                Vector2 Cdot    = vB + MathUtils.Cross(wB, m_rB) - vA - MathUtils.Cross(wA, m_rA);
                Vector2 impulse = m_mass.Solve22(-Cdot);

                m_impulse.X += impulse.X;
                m_impulse.Y += impulse.Y;

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
            Vector2 cA = data.Positions[m_indexA].Center;
            float   aA = data.Positions[m_indexA].Angle;
            Vector2 cB = data.Positions[m_indexB].Center;
            float   aB = data.Positions[m_indexB].Angle;

            Rotation
                qA = new Rotation(aA), qB = new Rotation(aB);

            float angularError  = 0.0f;
            float positionError = 0.0f;

            bool fixedRotation = (m_invIA + m_invIB == 0.0f);

            // Solve angular limit constraint.
            if (m_enableLimit && m_limitState != LimitState.InactiveLimit && fixedRotation == false)
            {
                float angle        = aB - aA - m_referenceAngle;
                float limitImpulse = 0.0f;

                if (m_limitState == LimitState.EqualLimits)
                {
                    // Prevent large angular corrections
                    float C = MathUtils.Clamp(
                        angle - m_lowerAngle,
                        -Settings.MaxAngularCorrection,
                        Settings.MaxAngularCorrection);
                    limitImpulse = -m_motorMass * C;
                    angularError = Math.Abs(C);
                }
                else if (m_limitState == LimitState.AtLowerLimit)
                {
                    float C = angle - m_lowerAngle;
                    angularError = -C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.Clamp(
                        C + Settings.AngularSlop,
                        -Settings.MaxAngularCorrection,
                        0.0f);
                    limitImpulse = -m_motorMass * C;
                }
                else if (m_limitState == LimitState.AtUpperLimit)
                {
                    float C = angle - m_upperAngle;
                    angularError = C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.Clamp(
                        C - Settings.AngularSlop,
                        0.0f,
                        Settings.MaxAngularCorrection);
                    limitImpulse = -m_motorMass * C;
                }

                aA -= m_invIA * limitImpulse;
                aB += m_invIB * limitImpulse;
            }

            // Solve point-to-point constraint.
            {
                qA.Set(aA);
                qB.Set(aB);
                Vector2 rA = MathUtils.Mul(qA, m_localAnchorA - m_localCenterA);
                Vector2 rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);

                Vector2 C = cB + rB - cA - rA;
                positionError = C.Length();

                float mA = m_invMassA, mB = m_invMassB;
                float iA = m_invIA,    iB = m_invIB;

                Matrix2x2 K = new Matrix2x2();
                K.ex.X = mA + mB + iA * rA.Y * rA.Y + iB * rB.Y * rB.Y;
                K.ex.Y = -iA * rA.X * rA.Y - iB * rB.X * rB.Y;
                K.ey.X = K.ex.Y;
                K.ey.Y = mA + mB + iA * rA.X * rA.X + iB * rB.X * rB.X;

                Vector2 impulse = -K.Solve(C);

                cA -= mA * impulse;
                aA -= iA * MathUtils.Cross(rA, impulse);

                cB += mB * impulse;
                aB += iB * MathUtils.Cross(rB, impulse);
            }

            data.Positions[m_indexA].Center = cA;
            data.Positions[m_indexA].Angle = aA;
            data.Positions[m_indexB].Center = cB;
            data.Positions[m_indexB].Angle = aB;

            return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }

        internal RevoluteJoint(RevoluteJointDef def) : base(def)
        {
            m_localAnchorA   = def.localAnchorA;
            m_localAnchorB   = def.localAnchorB;
            m_referenceAngle = def.referenceAngle;

            m_impulse.SetZero();
            m_motorImpulse = 0.0f;

            m_lowerAngle     = def.lowerAngle;
            m_upperAngle     = def.upperAngle;
            m_maxMotorTorque = def.maxMotorTorque;
            m_motorSpeed     = def.motorSpeed;
            m_enableLimit    = def.enableLimit;
            m_enableMotor    = def.enableMotor;
            m_limitState     = LimitState.InactiveLimit;
        }

        // Solver shared
        public Vector2 m_localAnchorA;

        public Vector2 m_localAnchorB;

        public Vector3 m_impulse;

        public float m_motorImpulse;

        public bool m_enableMotor;

        public float m_maxMotorTorque;

        public float m_motorSpeed;

        public bool m_enableLimit;

        public float m_referenceAngle;

        public float m_lowerAngle;

        public float m_upperAngle;

        // Solver temp
        public int m_indexA;

        public int m_indexB;

        public Vector2 m_rA;

        public Vector2 m_rB;

        public Vector2 m_localCenterA;

        public Vector2 m_localCenterB;

        public float m_invMassA;

        public float m_invMassB;

        public float m_invIA;

        public float m_invIB;

        public Matrix3x3 m_mass; // effective mass for point-to-point constraint.

        public float m_motorMass; // effective mass for motor/limit angular constraint.

        public LimitState m_limitState;
    };
}