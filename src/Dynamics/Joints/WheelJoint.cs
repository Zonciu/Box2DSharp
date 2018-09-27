using System;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// A wheel joint. This joint provides two degrees of freedom: translation
    /// along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
    /// line constraint with a rotational motor and a linear spring/damper.
    /// This joint is designed for vehicle suspensions.
    internal class WheelJoint : Joint
    {
        /// The local anchor point relative to bodyA's origin.
        ref Vector2 GetLocalAnchorA()
        {
            return ref m_localAnchorA;
        }

        /// The local anchor point relative to bodyB's origin.
        ref Vector2 GetLocalAnchorB()
        {
            return ref m_localAnchorB;
        }

        /// The local joint axis relative to bodyA.
        ref Vector2 GetLocalAxisA()
        {
            return ref m_localXAxisA;
        }

        /// Get the current joint translation, usually in meters.
        float GetJointTranslation()
        {
            var bA = BodyA;
            var bB = BodyB;

            Vector2 pA   = bA.GetWorldPoint(m_localAnchorA);
            Vector2 pB   = bB.GetWorldPoint(m_localAnchorB);
            Vector2 d    = pB - pA;
            Vector2 axis = bA.GetWorldVector(m_localXAxisA);

            float translation = MathUtils.Dot(d, axis);
            return translation;
        }

        /// Get the current joint linear speed, usually in meters per second.
        float GetJointLinearSpeed()
        {
            var bA = BodyA;
            var bB = BodyB;

            Vector2 rA   = MathUtils.Mul(bA._transform.Rotation, m_localAnchorA - bA._sweep.localCenter);
            Vector2 rB   = MathUtils.Mul(bB._transform.Rotation, m_localAnchorB - bB._sweep.localCenter);
            Vector2 p1   = bA._sweep.c + rA;
            Vector2 p2   = bB._sweep.c + rB;
            Vector2 d    = p2 - p1;
            Vector2 axis = MathUtils.Mul(bA._transform.Rotation, m_localXAxisA);

            Vector2 vA = bA._linearVelocity;
            Vector2 vB = bB._linearVelocity;
            float   wA = bA._angularVelocity;
            float   wB = bB._angularVelocity;

            float speed = MathUtils.Dot(d, MathUtils.Cross(wA, axis))
                        + MathUtils.Dot(axis, vB + MathUtils.Cross(wB, rB) - vA - MathUtils.Cross(wA, rA));
            return speed;
        }

        /// Get the current joint angle in radians.
        float GetJointAngle()
        {
            var bA = BodyA;
            var bB = BodyB;
            return bB._sweep.a - bA._sweep.a;
        }

        /// Get the current joint angular speed in radians per second.
        float GetJointAngularSpeed()
        {
            float wA = BodyA._angularVelocity;
            float wB = BodyB._angularVelocity;
            return wB - wA;
        }

        /// Is the joint motor enabled?
        bool IsMotorEnabled()
        {
            return m_enableMotor;
        }

        /// Enable/disable the joint motor.
        void EnableMotor(bool flag)
        {
            if (flag != m_enableMotor)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                m_enableMotor   = flag;
            }
        }

        /// Set the motor speed, usually in radians per second.
        void SetMotorSpeed(float speed)
        {
            if (!speed.Equals(m_motorSpeed))
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                m_motorSpeed    = speed;
            }
        }

        /// Get the motor speed, usually in radians per second.
        float GetMotorSpeed()
        {
            return m_motorSpeed;
        }

        /// Set/Get the maximum motor force, usually in N-m.
        void SetMaxMotorTorque(float torque)
        {
            if (torque != m_maxMotorTorque)
            {
                BodyA.IsAwake  = true;
                BodyB.IsAwake  = true;
                m_maxMotorTorque = torque;
            }
        }

        float GetMaxMotorTorque()
        {
            return m_maxMotorTorque;
        }

        /// Get the current motor torque given the inverse time step, usually in N-m.
        float GetMotorTorque(float inv_dt)
        {
            return inv_dt * m_motorImpulse;
        }

        /// Set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring.
        void SetSpringFrequencyHz(float hz)
        {
            m_frequencyHz = hz;
        }

        float GetSpringFrequencyHz()
        {
            return m_frequencyHz;
        }

        /// Set/Get the spring damping ratio
        void SetSpringDampingRatio(float ratio)
        {
            m_dampingRatio = ratio;
        }

        float GetSpringDampingRatio()
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
            return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
        }

        /// <inheritdoc />
        public override float GetReactionTorque(float inv_dt)
        {
            return inv_dt * m_motorImpulse;
        }

        /// Dump to Logger.Log
        public override void Dump()
        {
            var indexA = BodyA._islandIndex;
            var indexB = BodyB._islandIndex;

            Logger.Log("  b2WheelJointDef jd;");
            Logger.Log($"  jd.bodyA = bodies[{indexA}];");
            Logger.Log($"  jd.bodyB = bodies[{indexB}];");
            Logger.Log($"  jd.collideConnected = bool({CollideConnected});");
            Logger.Log($"  jd.localAnchorA.Set({m_localAnchorA.X}, {m_localAnchorA.Y});");
            Logger.Log($"  jd.localAnchorB.Set({m_localAnchorB.X}, {m_localAnchorB.Y});");
            Logger.Log($"  jd.localAxisA.Set({m_localXAxisA.X}, {m_localXAxisA.Y});");
            Logger.Log($"  jd.enableMotor = bool({m_enableMotor});");
            Logger.Log($"  jd.motorSpeed = {m_motorSpeed};");
            Logger.Log($"  jd.maxMotorTorque = {m_maxMotorTorque};");
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

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA,    iB = m_invIB;

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

            // Compute the effective masses.
            Vector2 rA = MathUtils.Mul(qA, m_localAnchorA - m_localCenterA);
            Vector2 rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);
            Vector2 d  = cB + rB - cA - rA;

            // Point to line constraint
            {
                m_ay  = MathUtils.Mul(qA, m_localYAxisA);
                m_sAy = MathUtils.Cross(d + rA, m_ay);
                m_sBy = MathUtils.Cross(rB, m_ay);

                m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;

                if (m_mass > 0.0f)
                {
                    m_mass = 1.0f / m_mass;
                }
            }

            // Spring constraint
            m_springMass = 0.0f;
            m_bias       = 0.0f;
            m_gamma      = 0.0f;
            if (m_frequencyHz > 0.0f)
            {
                m_ax  = MathUtils.Mul(qA, m_localXAxisA);
                m_sAx = MathUtils.Cross(d + rA, m_ax);
                m_sBx = MathUtils.Cross(rB, m_ax);

                float invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;

                if (invMass > 0.0f)
                {
                    m_springMass = 1.0f / invMass;

                    float C = MathUtils.Dot(d, m_ax);

                    // Frequency
                    float omega = 2.0f * Settings.Pi * m_frequencyHz;

                    // Damping coefficient
                    float damp = 2.0f * m_springMass * m_dampingRatio * omega;

                    // Spring stiffness
                    float k = m_springMass * omega * omega;

                    // magic formulas
                    float h = data.Step.dt;
                    m_gamma = h * (damp + h * k);
                    if (m_gamma > 0.0f)
                    {
                        m_gamma = 1.0f / m_gamma;
                    }

                    m_bias = C * h * k * m_gamma;

                    m_springMass = invMass + m_gamma;
                    if (m_springMass > 0.0f)
                    {
                        m_springMass = 1.0f / m_springMass;
                    }
                }
            }
            else
            {
                m_springImpulse = 0.0f;
            }

            // Rotational motor
            if (m_enableMotor)
            {
                m_motorMass = iA + iB;
                if (m_motorMass > 0.0f)
                {
                    m_motorMass = 1.0f / m_motorMass;
                }
            }
            else
            {
                m_motorMass    = 0.0f;
                m_motorImpulse = 0.0f;
            }

            if (data.Step.warmStarting)
            {
                // Account for variable time step.
                m_impulse       *= data.Step.dtRatio;
                m_springImpulse *= data.Step.dtRatio;
                m_motorImpulse  *= data.Step.dtRatio;

                Vector2 P  = m_impulse * m_ay + m_springImpulse * m_ax;
                float   LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
                float   LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;

                vA -= m_invMassA * P;
                wA -= m_invIA * LA;

                vB += m_invMassB * P;
                wB += m_invIB * LB;
            }
            else
            {
                m_impulse       = 0.0f;
                m_springImpulse = 0.0f;
                m_motorImpulse  = 0.0f;
            }

            data.Velocities[m_indexA].v = vA;
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v = vB;
            data.Velocities[m_indexB].w = wB;
        }

        /// <inheritdoc />
        internal override void SolveVelocityConstraints(SolverData data)
        {
            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA,    iB = m_invIB;

            Vector2 vA = data.Velocities[m_indexA].v;
            float   wA = data.Velocities[m_indexA].w;
            Vector2 vB = data.Velocities[m_indexB].v;
            float   wB = data.Velocities[m_indexB].w;

            // Solve spring constraint
            {
                float Cdot    = MathUtils.Dot(m_ax, vB - vA) + m_sBx * wB - m_sAx * wA;
                float impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
                m_springImpulse += impulse;

                Vector2 P  = impulse * m_ax;
                float   LA = impulse * m_sAx;
                float   LB = impulse * m_sBx;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

            // Solve rotational motor constraint
            {
                float Cdot    = wB - wA - m_motorSpeed;
                float impulse = -m_motorMass * Cdot;

                float oldImpulse = m_motorImpulse;
                float maxImpulse = data.Step.dt * m_maxMotorTorque;
                m_motorImpulse = MathUtils.Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse        = m_motorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve point to line constraint
            {
                float Cdot    = MathUtils.Dot(m_ay, vB - vA) + m_sBy * wB - m_sAy * wA;
                float impulse = -m_mass * Cdot;
                m_impulse += impulse;

                Vector2 P  = impulse * m_ay;
                float   LA = impulse * m_sAy;
                float   LB = impulse * m_sBy;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
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

            Vector2 rA = MathUtils.Mul(qA, m_localAnchorA - m_localCenterA);
            Vector2 rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);
            Vector2 d  = (cB - cA) + rB - rA;

            Vector2 ay = MathUtils.Mul(qA, m_localYAxisA);

            float sAy = MathUtils.Cross(d + rA, ay);
            float sBy = MathUtils.Cross(rB, ay);

            float C = MathUtils.Dot(d, ay);

            float k = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

            float impulse;
            if (k != 0.0f)
            {
                impulse = -C / k;
            }
            else
            {
                impulse = 0.0f;
            }

            Vector2 P  = impulse * ay;
            float   LA = impulse * sAy;
            float   LB = impulse * sBy;

            cA -= m_invMassA * P;
            aA -= m_invIA * LA;
            cB += m_invMassB * P;
            aB += m_invIB * LB;

            data.Positions[m_indexA].Center = cA;
            data.Positions[m_indexA].Angle = aA;
            data.Positions[m_indexB].Center = cB;
            data.Positions[m_indexB].Angle = aB;

            return Math.Abs(C) <= Settings.LinearSlop;
        }

        internal WheelJoint(WheelJointDef def) : base(def)
        {
            m_localAnchorA = def.localAnchorA;
            m_localAnchorB = def.localAnchorB;
            m_localXAxisA  = def.localAxisA;
            m_localYAxisA  = MathUtils.Cross(1.0f, m_localXAxisA);

            m_mass          = 0.0f;
            m_impulse       = 0.0f;
            m_motorMass     = 0.0f;
            m_motorImpulse  = 0.0f;
            m_springMass    = 0.0f;
            m_springImpulse = 0.0f;

            m_maxMotorTorque = def.maxMotorTorque;
            m_motorSpeed     = def.motorSpeed;
            m_enableMotor    = def.enableMotor;

            m_frequencyHz  = def.frequencyHz;
            m_dampingRatio = def.dampingRatio;

            m_bias  = 0.0f;
            m_gamma = 0.0f;

            m_ax.SetZero();
            m_ay.SetZero();
        }

        float m_frequencyHz;

        float m_dampingRatio;

        // Solver shared
        Vector2 m_localAnchorA;

        Vector2 m_localAnchorB;

        Vector2 m_localXAxisA;

        Vector2 m_localYAxisA;

        float m_impulse;

        float m_motorImpulse;

        float m_springImpulse;

        float m_maxMotorTorque;

        float m_motorSpeed;

        bool m_enableMotor;

        // Solver temp
        int m_indexA;

        int m_indexB;

        Vector2 m_localCenterA;

        Vector2 m_localCenterB;

        float m_invMassA;

        float m_invMassB;

        float m_invIA;

        float m_invIB;

        Vector2 m_ax, m_ay;

        float m_sAx, m_sBx;

        float m_sAy, m_sBy;

        float m_mass;

        float m_motorMass;

        float m_springMass;

        float m_bias;

        float m_gamma;
    };
}