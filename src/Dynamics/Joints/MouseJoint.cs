using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// A mouse joint is used to make a point on a body track a
    /// specified world point. This a soft constraint with a maximum
    /// force. This allows the constraint to stretch and without
    /// applying huge forces.
    /// NOTE: this joint is not documented in the manual because it was
    /// developed to be used in the testbed. If you want to learn how to
    /// use the mouse joint, look at the testbed.
    public class MouseJoint : Joint
    {
        /// Implements b2Joint.
        /// Use this to update the target point.
        void SetTarget(in Vector2 target)
        {
            if (target != m_targetA)
            {
                BodyB.IsAwake = true;
                m_targetA       = target;
            }
        }

        ref readonly Vector2 GetTarget()
        {
            return ref m_targetA;
        }

        /// Set/get the maximum force in Newtons.
        void SetMaxForce(float force)
        {
            m_maxForce = force;
        }

        float GetMaxForce()
        {
            return m_maxForce;
        }

        /// Set/get the frequency in Hertz.
        void SetFrequency(float hz)
        {
            m_frequencyHz = hz;
        }

        float GetFrequency()
        {
            return m_frequencyHz;
        }

        /// Set/get the damping ratio (dimensionless).
        void SetDampingRatio(float ratio)
        {
            m_dampingRatio = ratio;
        }

        float GetDampingRatio()
        {
            return m_dampingRatio;
        }

        /// <inheritdoc />
        public override void ShiftOrigin(in Vector2 newOrigin)
        {
            m_targetA -= newOrigin;
        }

        /// <inheritdoc />
        public override Vector2 GetAnchorA()
        {
            return m_targetA;
        }

        /// <inheritdoc />
        public override Vector2 GetAnchorB()
        {
            return BodyB.GetWorldPoint(m_localAnchorB);
        }

        /// <inheritdoc />
        public override Vector2 GetReactionForce(float inv_dt)
        {
            return inv_dt * m_impulse;
        }

        /// <inheritdoc />
        public override float GetReactionTorque(float inv_dt)
        {
            return inv_dt * 0.0f;
        }

        /// The mouse joint does not support dumping.
        public override void Dump()
        {
            Logger.Log("Mouse joint dumping is not supported.");
        }

        /// <inheritdoc />
        internal override void InitVelocityConstraints(SolverData data)
        {
            m_indexB       = BodyB._islandIndex;
            m_localCenterB = BodyB._sweep.localCenter;
            m_invMassB     = BodyB._invMass;
            m_invIB        = BodyB._inverseInertia;

            Vector2 cB = data.Positions[m_indexB].Center;
            float   aB = data.Positions[m_indexB].Angle;
            Vector2 vB = data.Velocities[m_indexB].v;
            float   wB = data.Velocities[m_indexB].w;

            var qB = new Rotation(aB);

            float mass = BodyB.Mass;

            // Frequency
            float omega = 2.0f * Settings.Pi * m_frequencyHz;

            // Damping coefficient
            float d = 2.0f * mass * m_dampingRatio * omega;

            // Spring stiffness
            float k = mass * (omega * omega);

            // magic formulas
            // gamma has units of inverse mass.
            // beta has units of inverse time.
            float h = data.Step.dt;
            Debug.Assert(d + h * k > Settings.Epsilon);
            m_gamma = h * (d + h * k);
            if (m_gamma != 0.0f)
            {
                m_gamma = 1.0f / m_gamma;
            }

            m_beta = h * k * m_gamma;

            // Compute the effective mass matrix.
            m_rB = MathUtils.Mul(qB, m_localAnchorB - m_localCenterB);

            // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
            //      = [1/m1+1/m2     0    ] + invI1 * [r1.Y*r1.Y -r1.X*r1.Y] + invI2 * [r1.Y*r1.Y -r1.X*r1.Y]
            //        [    0     1/m1+1/m2]           [-r1.X*r1.Y r1.X*r1.X]           [-r1.X*r1.Y r1.X*r1.X]
            Matrix2x2 K = new Matrix2x2();
            K.ex.X = m_invMassB + m_invIB * m_rB.Y * m_rB.Y + m_gamma;
            K.ex.Y = -m_invIB * m_rB.X * m_rB.Y;
            K.ey.X = K.ex.Y;
            K.ey.Y = m_invMassB + m_invIB * m_rB.X * m_rB.X + m_gamma;

            m_mass = K.GetInverse();

            m_C =  cB + m_rB - m_targetA;
            m_C *= m_beta;

            // Cheat with some damping
            wB *= 0.98f;

            if (data.Step.warmStarting)
            {
                m_impulse *= data.Step.dtRatio;
                vB        += m_invMassB * m_impulse;
                wB        += m_invIB * MathUtils.Cross(m_rB, m_impulse);
            }
            else
            {
                m_impulse.SetZero();
            }

            data.Velocities[m_indexB].v = vB;
            data.Velocities[m_indexB].w = wB;
        }

        /// <inheritdoc />
        internal override void SolveVelocityConstraints(SolverData data)
        {
            Vector2 vB = data.Velocities[m_indexB].v;
            float   wB = data.Velocities[m_indexB].w;

            // Cdot = v + cross(w, r)
            Vector2 Cdot    = vB + MathUtils.Cross(wB, m_rB);
            Vector2 impulse = MathUtils.Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));

            Vector2 oldImpulse = m_impulse;
            m_impulse += impulse;
            float maxImpulse = data.Step.dt * m_maxForce;
            if (m_impulse.LengthSquared() > maxImpulse * maxImpulse)
            {
                m_impulse *= maxImpulse / m_impulse.Length();
            }

            impulse = m_impulse - oldImpulse;

            vB += m_invMassB * impulse;
            wB += m_invIB * MathUtils.Cross(m_rB, impulse);

            data.Velocities[m_indexB].v = vB;
            data.Velocities[m_indexB].w = wB;
        }

        /// <inheritdoc />
        internal override bool SolvePositionConstraints(SolverData data)
        {
            return true;
        }

        internal MouseJoint(MouseJointDef def) : base(def)
        {
            Debug.Assert(def.target.IsValid());
            Debug.Assert(def.maxForce.IsValid() && def.maxForce >= 0.0f);
            Debug.Assert(def.frequencyHz.IsValid() && def.frequencyHz >= 0.0f);
            Debug.Assert(def.dampingRatio.IsValid() && def.dampingRatio >= 0.0f);

            m_targetA      = def.target;
            m_localAnchorB = MathUtils.MulT(BodyB.GetTransform(), m_targetA);

            m_maxForce = def.maxForce;
            m_impulse.SetZero();

            m_frequencyHz  = def.frequencyHz;
            m_dampingRatio = def.dampingRatio;

            m_beta  = 0.0f;
            m_gamma = 0.0f;
        }

        Vector2 m_localAnchorB;

        Vector2 m_targetA;

        float m_frequencyHz;

        float m_dampingRatio;

        float m_beta;

        // Solver shared
        Vector2 m_impulse;

        float m_maxForce;

        float m_gamma;

        // Solver temp
        int m_indexA;

        int m_indexB;

        Vector2 m_rB;

        Vector2 m_localCenterB;

        float m_invMassB;

        float m_invIB;

        Matrix2x2 m_mass;

        Vector2 m_C;
    };
}