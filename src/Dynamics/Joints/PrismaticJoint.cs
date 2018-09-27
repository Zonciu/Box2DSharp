using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// A prismatic joint. This joint provides one degree of freedom: translation
    /// along an axis fixed in bodyA. Relative rotation is prevented. You can
    /// use a joint limit to restrict the range of motion and a joint motor to
    /// drive the motion or to model joint friction.
    ///
    /// Linear constraint (point-to-line)
    /// d = p2 - p1 = x2 + r2 - x1 - r1
    /// C = dot(perp, d)
    /// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    ///      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
    /// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
    ///
    /// Angular constraint
    /// C = a2 - a1 + a_initial
    /// Cdot = w2 - w1
    /// J = [0 0 -1 0 0 1]
    ///
    /// K = J * invM * JT
    ///
    /// J = [-a -s1 a s2]
    ///     [0  -1  0  1]
    /// a = perp
    /// s1 = cross(d + r1, a) = cross(p2 - x1, a)
    /// s2 = cross(r2, a) = cross(p2 - x2, a)
    /// Motor/Limit linear constraint
    /// C = dot(ax1, d)
    /// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
    /// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]
    /// Block Solver
    /// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
    /// when the mass has poor distribution (leading to large torques about the joint anchor points).
    ///
    /// The Jacobian has 3 rows:
    /// J = [-uT -s1 uT s2] // linear
    ///     [0   -1   0  1] // angular
    ///     [-vT -a1 vT a2] // limit
    ///
    /// u = perp
    /// v = axis
    /// s1 = cross(d + r1, u), s2 = cross(r2, u)
    /// a1 = cross(d + r1, v), a2 = cross(r2, v)
    /// M * (v2 - v1) = JT * df
    /// J * v2 = bias
    ///
    /// v2 = v1 + invM * JT * df
    /// J * (v1 + invM * JT * df) = bias
    /// K * df = bias - J * v1 = -Cdot
    /// K = J * invM * JT
    /// Cdot = J * v1 - bias
    ///
    /// Now solve for f2.
    /// df = f2 - f1
    /// K * (f2 - f1) = -Cdot
    /// f2 = invK * (-Cdot) + f1
    ///
    /// Clamp accumulated limit impulse.
    /// lower: f2(3) = max(f2(3), 0)
    /// upper: f2(3) = min(f2(3), 0)
    ///
    /// Solve for correct f2(1:2)
    /// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
    ///                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
    /// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
    /// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
    ///
    /// Now compute impulse to be applied:
    /// df = f2 - f1
    internal class PrismaticJoint : Joint
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

        /// The local joint axis relative to bodyA.
        public ref readonly Vector2 GetLocalAxisA()
        {
            return ref m_localXAxisA;
        }

        /// Get the reference angle.
        public float GetReferenceAngle()
        {
            return m_referenceAngle;
        }

        /// Get the current joint translation, usually in meters.
        public float GetJointTranslation()
        {
            Vector2 pA   = BodyA.GetWorldPoint(m_localAnchorA);
            Vector2 pB   = BodyB.GetWorldPoint(m_localAnchorB);
            Vector2 d    = pB - pA;
            Vector2 axis = BodyA.GetWorldVector(m_localXAxisA);

            float translation = MathUtils.Dot(d, axis);
            return translation;
        }

        /// Get the current joint translation speed, usually in meters per second.
        public float GetJointSpeed()
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

        /// Get the lower joint limit, usually in meters.
        float GetLowerLimit()
        {
            return m_lowerTranslation;
        }

        /// Get the upper joint limit, usually in meters.
        float GetUpperLimit()
        {
            return m_upperTranslation;
        }

        /// Set the joint limits, usually in meters.
        void SetLimits(float lower, float upper)
        {
            Debug.Assert(lower <= upper);
            if (lower != m_lowerTranslation || upper != m_upperTranslation)
            {
                BodyA.IsAwake    = true;
                BodyB.IsAwake    = true;
                m_lowerTranslation = lower;
                m_upperTranslation = upper;
                m_impulse.Z        = 0.0f;
            }
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

        /// Set the motor speed, usually in meters per second.
        void SetMotorSpeed(float speed)
        {
            if (speed != m_motorSpeed)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                m_motorSpeed    = speed;
            }
        }

        /// Get the motor speed, usually in meters per second.
        float GetMotorSpeed()
        {
            return m_motorSpeed;
        }

        /// Set the maximum motor force, usually in N.
        void SetMaxMotorForce(float force)
        {
            if (force != m_maxMotorForce)
            {
                BodyA.IsAwake = true;
                BodyB.IsAwake = true;
                m_maxMotorForce = force;
            }
        }

        float GetMaxMotorForce()
        {
            return m_maxMotorForce;
        }

        /// Get the current motor force given the inverse time step, usually in N.
        float GetMotorForce(float inv_dt)
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
            return inv_dt * (m_impulse.X * m_perp + (m_motorImpulse + m_impulse.Z) * m_axis);
        }

        /// <inheritdoc />
        public override float GetReactionTorque(float inv_dt)
        {
            return inv_dt * m_impulse.Y;
        }

        /// Dump to b2Log
        public override void Dump()
        { }

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

            // Compute the effective masses.
            Vector2 rA = MathUtils.Mul(qA, m_localAnchorA - m_localCenterA);
            Vector2 rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);
            Vector2 d  = (cB - cA) + rB - rA;

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA,    iB = m_invIB;

            // Compute motor Jacobian and effective mass.
            {
                m_axis = MathUtils.Mul(qA, m_localXAxisA);
                m_a1   = MathUtils.Cross(d + rA, m_axis);
                m_a2   = MathUtils.Cross(rB, m_axis);

                m_motorMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
                if (m_motorMass > 0.0f)
                {
                    m_motorMass = 1.0f / m_motorMass;
                }
            }

            // Prismatic constraint.
            {
                m_perp = MathUtils.Mul(qA, m_localYAxisA);

                m_s1 = MathUtils.Cross(d + rA, m_perp);
                m_s2 = MathUtils.Cross(rB, m_perp);

                float k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
                float k12 = iA * m_s1 + iB * m_s2;
                float k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
                float k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    // For bodies with fixed rotation.
                    k22 = 1.0f;
                }

                float k23 = iA * m_a1 + iB * m_a2;
                float k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;

                m_K.ex.Set(k11, k12, k13);
                m_K.ey.Set(k12, k22, k23);
                m_K.ez.Set(k13, k23, k33);
            }

            // Compute motor and limit terms.
            if (m_enableLimit)
            {
                float jointTranslation = MathUtils.Dot(m_axis, d);
                if (Math.Abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.LinearSlop)
                {
                    m_limitState = LimitState.EqualLimits;
                }
                else if (jointTranslation <= m_lowerTranslation)
                {
                    if (m_limitState != LimitState.AtLowerLimit)
                    {
                        m_limitState = LimitState.AtLowerLimit;
                        m_impulse.Z  = 0.0f;
                    }
                }
                else if (jointTranslation >= m_upperTranslation)
                {
                    if (m_limitState != LimitState.AtUpperLimit)
                    {
                        m_limitState = LimitState.AtUpperLimit;
                        m_impulse.Z  = 0.0f;
                    }
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
                m_impulse.Z  = 0.0f;
            }

            if (m_enableMotor == false)
            {
                m_motorImpulse = 0.0f;
            }

            if (data.Step.warmStarting)
            {
                // Account for variable time step.
                m_impulse      *= data.Step.dtRatio;
                m_motorImpulse *= data.Step.dtRatio;

                Vector2 P  = m_impulse.X * m_perp + (m_motorImpulse + m_impulse.Z) * m_axis;
                float   LA = m_impulse.X * m_s1 + m_impulse.Y + (m_motorImpulse + m_impulse.Z) * m_a1;
                float   LB = m_impulse.X * m_s2 + m_impulse.Y + (m_motorImpulse + m_impulse.Z) * m_a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
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

        internal override void SolveVelocityConstraints(SolverData data)
        {
            Vector2 vA = data.Velocities[m_indexA].v;
            float   wA = data.Velocities[m_indexA].w;
            Vector2 vB = data.Velocities[m_indexB].v;
            float   wB = data.Velocities[m_indexB].w;

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA,    iB = m_invIB;

            // Solve linear motor constraint.
            if (m_enableMotor && m_limitState != LimitState.EqualLimits)
            {
                float Cdot       = MathUtils.Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
                float impulse    = m_motorMass * (m_motorSpeed - Cdot);
                float oldImpulse = m_motorImpulse;
                float maxImpulse = data.Step.dt * m_maxMotorForce;
                m_motorImpulse = MathUtils.Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse        = m_motorImpulse - oldImpulse;

                Vector2 P  = impulse * m_axis;
                float   LA = impulse * m_a1;
                float   LB = impulse * m_a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

            Vector2 Cdot1;
            Cdot1.X = MathUtils.Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
            Cdot1.Y = wB - wA;

            if (m_enableLimit && m_limitState != LimitState.InactiveLimit)
            {
                // Solve prismatic and limit constraint in block form.
                float Cdot2;
                Cdot2 = MathUtils.Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
                var Cdot = new Vector3(Cdot1.X, Cdot1.Y, Cdot2);

                Vector3 f1 = m_impulse;
                Vector3 df = m_K.Solve33(-Cdot);
                m_impulse += df;

                if (m_limitState == LimitState.AtLowerLimit)
                {
                    m_impulse.Z = Math.Max(m_impulse.Z, 0.0f);
                }
                else if (m_limitState == LimitState.AtUpperLimit)
                {
                    m_impulse.Z = Math.Min(m_impulse.Z, 0.0f);
                }

                // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
                Vector2 b   = -Cdot1 - (m_impulse.Z - f1.Z) * new Vector2(m_K.ez.X, m_K.ez.Y);
                Vector2 f2r = m_K.Solve22(b) + new Vector2(f1.X, f1.Y);
                m_impulse.X = f2r.X;
                m_impulse.Y = f2r.Y;

                df = m_impulse - f1;

                Vector2 P  = df.X * m_perp + df.Z * m_axis;
                float   LA = df.X * m_s1 + df.Y + df.Z * m_a1;
                float   LB = df.X * m_s2 + df.Y + df.Z * m_a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }
            else
            {
                // Limit is inactive, just solve the prismatic constraint in block form.
                Vector2 df = m_K.Solve22(-Cdot1);
                m_impulse.X += df.X;
                m_impulse.Y += df.Y;

                Vector2 P  = df.X * m_perp;
                float   LA = df.X * m_s1 + df.Y;
                float   LB = df.X * m_s2 + df.Y;

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

        // A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
        // the position solver is not there to resolve forces.It is only there to cope with integration error.
        //
        // Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
        //
        // We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
        // solver indicates the limit is inactive.
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

            // Compute fresh Jacobians
            Vector2 rA = MathUtils.Mul(qA, m_localAnchorA - m_localCenterA);
            Vector2 rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);
            Vector2 d  = cB + rB - cA - rA;

            Vector2 axis = MathUtils.Mul(qA, m_localXAxisA);
            float   a1   = MathUtils.Cross(d + rA, axis);
            float   a2   = MathUtils.Cross(rB, axis);
            Vector2 perp = MathUtils.Mul(qA, m_localYAxisA);

            float s1 = MathUtils.Cross(d + rA, perp);
            float s2 = MathUtils.Cross(rB, perp);

            Vector3 impulse = new Vector3();
            Vector2 C1      = new Vector2();
            C1.X = MathUtils.Dot(perp, d);
            C1.Y = aB - aA - m_referenceAngle;

            float linearError  = Math.Abs(C1.X);
            float angularError = Math.Abs(C1.Y);

            bool  active = false;
            float C2     = 0.0f;
            if (m_enableLimit)
            {
                float translation = MathUtils.Dot(axis, d);
                if (Math.Abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.LinearSlop)
                {
                    // Prevent large angular corrections
                    C2 = MathUtils.Clamp(
                        translation,
                        -Settings.MaxLinearCorrection,
                        Settings.MaxLinearCorrection);
                    linearError = Math.Max(linearError, Math.Abs(translation));
                    active      = true;
                }
                else if (translation <= m_lowerTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(
                        translation - m_lowerTranslation + Settings.LinearSlop,
                        -Settings.MaxLinearCorrection,
                        0.0f);
                    linearError = Math.Max(linearError, m_lowerTranslation - translation);
                    active      = true;
                }
                else if (translation >= m_upperTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(
                        translation - m_upperTranslation - Settings.LinearSlop,
                        0.0f,
                        Settings.MaxLinearCorrection);
                    linearError = Math.Max(linearError, translation - m_upperTranslation);
                    active      = true;
                }
            }

            if (active)
            {
                float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                float k12 = iA * s1 + iB * s2;
                float k13 = iA * s1 * a1 + iB * s2 * a2;
                float k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    // For fixed rotation
                    k22 = 1.0f;
                }

                float k23 = iA * a1 + iB * a2;
                float k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

                Matrix3x3 K = new Matrix3x3();
                K.ex.Set(k11, k12, k13);
                K.ey.Set(k12, k22, k23);
                K.ez.Set(k13, k23, k33);

                Vector3 C = new Vector3();
                C.X = C1.X;
                C.Y = C1.Y;
                C.Z = C2;

                impulse = K.Solve33(-C);
            }
            else
            {
                float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                float k12 = iA * s1 + iB * s2;
                float k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    k22 = 1.0f;
                }

                Matrix2x2 K = new Matrix2x2();
                K.ex.Set(k11, k12);
                K.ey.Set(k12, k22);

                Vector2 impulse1 = K.Solve(-C1);
                impulse.X = impulse1.X;
                impulse.Y = impulse1.Y;
                impulse.Z = 0.0f;
            }

            Vector2 P  = impulse.X * perp + impulse.Z * axis;
            float   LA = impulse.X * s1 + impulse.Y + impulse.Z * a1;
            float   LB = impulse.X * s2 + impulse.Y + impulse.Z * a2;

            cA -= mA * P;
            aA -= iA * LA;
            cB += mB * P;
            aB += iB * LB;

            data.Positions[m_indexA].Center = cA;
            data.Positions[m_indexA].Angle = aA;
            data.Positions[m_indexB].Center = cB;
            data.Positions[m_indexB].Angle = aB;

            return linearError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }

        internal PrismaticJoint(PrismaticJointDef def) : base(def)
        {
            m_localAnchorA = def.localAnchorA;
            m_localAnchorB = def.localAnchorB;
            m_localXAxisA  = def.localAxisA;
            m_localXAxisA.Normalize();
            m_localYAxisA    = MathUtils.Cross(1.0f, m_localXAxisA);
            m_referenceAngle = def.referenceAngle;

            m_impulse.SetZero();
            m_motorMass    = 0.0f;
            m_motorImpulse = 0.0f;

            m_lowerTranslation = def.lowerTranslation;
            m_upperTranslation = def.upperTranslation;
            m_maxMotorForce    = def.maxMotorForce;
            m_motorSpeed       = def.motorSpeed;
            m_enableLimit      = def.enableLimit;
            m_enableMotor      = def.enableMotor;
            m_limitState       = LimitState.InactiveLimit;

            m_axis.SetZero();
            m_perp.SetZero();
        }

        // Solver shared
        internal Vector2 m_localAnchorA;

        internal Vector2 m_localAnchorB;

        internal Vector2 m_localXAxisA;

        internal Vector2 m_localYAxisA;

        internal float m_referenceAngle;

        internal Vector3 m_impulse;

        internal float m_motorImpulse;

        internal float m_lowerTranslation;

        internal float m_upperTranslation;

        internal float m_maxMotorForce;

        internal float m_motorSpeed;

        internal bool m_enableLimit;

        internal bool m_enableMotor;

        internal LimitState m_limitState;

        // Solver temp
        internal int m_indexA;

        internal int m_indexB;

        internal Vector2 m_localCenterA;

        internal Vector2 m_localCenterB;

        internal float m_invMassA;

        internal float m_invMassB;

        internal float m_invIA;

        internal float m_invIB;

        internal Vector2 m_axis, m_perp;

        internal float m_s1, m_s2;

        internal float m_a1, m_a2;

        internal Matrix3x3 m_K;

        internal float m_motorMass;
    };
}