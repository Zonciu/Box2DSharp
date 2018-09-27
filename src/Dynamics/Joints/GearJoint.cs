using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// A gear joint is used to connect two joints together. Either joint
    /// can be a revolute or prismatic joint. You specify a gear ratio
    /// to bind the motions together:
    /// coordinate1 + ratio * coordinate2 = constant
    /// The ratio can be negative or positive. If one joint is a revolute joint
    /// and the other joint is a prismatic joint, then the ratio will have units
    /// of length or units of 1/length.
    /// @warning You have to manually destroy the gear joint if joint1 or joint2
    /// is destroyed.
    public class GearJoint : Joint
    {
        // Body A is connected to body C
        // Body B is connected to body D
        internal readonly Body BodyC;

        internal readonly Body BodyD;

        internal readonly float Constant;

        private float _iA, _iB, _iC, _iD;

        private float _impulse;

        // Solver temp
        private int _indexA, _indexB, _indexC, _indexD;

        private readonly Joint _joint1;

        private readonly Joint _joint2;

        private Vector2 _JvAC, _JvBD;

        private float _jwA, _jwB, _jwC, _jwD;

        private Vector2 _lcA, _lcB, _lcC, _lcD;

        // Solver shared
        private readonly Vector2 _localAnchorA;

        private readonly Vector2 _localAnchorB;

        private readonly Vector2 _localAnchorC;

        private readonly Vector2 _localAnchorD;

        private readonly Vector2 _localAxisC;

        private readonly Vector2 _localAxisD;

        private float _mA, _mB, _mC, _mD;

        private float _mass;

        private float _ratio;

        private readonly float _referenceAngleA;

        private readonly float _referenceAngleB;

        private readonly JointType _typeA;

        private readonly JointType _typeB;

        public GearJoint(GearJointDef def) : base(def)
        {
            _joint1 = def.joint1;
            _joint2 = def.joint2;

            _typeA = _joint1.JointType;
            _typeB = _joint2.JointType;

            Debug.Assert(_typeA == JointType.RevoluteJoint || _typeA == JointType.PrismaticJoint);
            Debug.Assert(_typeB == JointType.RevoluteJoint || _typeB == JointType.PrismaticJoint);

            float coordinateA, coordinateB;

            // TODO_ERIN there might be some problem with the joint edges in b2Joint.

            BodyC = _joint1.BodyA;
            BodyA = _joint1.BodyB;

            // Get geometry of joint1
            var xfA = BodyA._transform;
            var aA  = BodyA._sweep.a;
            var xfC = BodyC._transform;
            var aC  = BodyC._sweep.a;

            if (_typeA == JointType.RevoluteJoint)
            {
                var revolute = (RevoluteJoint) def.joint1;
                _localAnchorC    = revolute.m_localAnchorA;
                _localAnchorA    = revolute.m_localAnchorB;
                _referenceAngleA = revolute.m_referenceAngle;
                _localAxisC.SetZero();

                coordinateA = aA - aC - _referenceAngleA;
            }
            else
            {
                var prismatic = (PrismaticJoint) def.joint1;
                _localAnchorC    = prismatic.m_localAnchorA;
                _localAnchorA    = prismatic.m_localAnchorB;
                _referenceAngleA = prismatic.m_referenceAngle;
                _localAxisC      = prismatic.m_localXAxisA;

                var pC = _localAnchorC;
                var pA = MathUtils.MulT(
                    xfC.Rotation,
                    MathUtils.Mul(xfA.Rotation, _localAnchorA) + (xfA.Position - xfC.Position));
                coordinateA = MathUtils.Dot(pA - pC, _localAxisC);
            }

            BodyD = _joint2.BodyA;
            BodyB = _joint2.BodyB;

            // Get geometry of joint2
            var xfB = BodyB._transform;
            var aB  = BodyB._sweep.a;
            var xfD = BodyD._transform;
            var aD  = BodyD._sweep.a;

            if (_typeB == JointType.RevoluteJoint)
            {
                var revolute = (RevoluteJoint) def.joint2;
                _localAnchorD    = revolute.m_localAnchorA;
                _localAnchorB    = revolute.m_localAnchorB;
                _referenceAngleB = revolute.m_referenceAngle;
                _localAxisD.SetZero();

                coordinateB = aB - aD - _referenceAngleB;
            }
            else
            {
                var prismatic = (PrismaticJoint) def.joint2;
                _localAnchorD    = prismatic.m_localAnchorA;
                _localAnchorB    = prismatic.m_localAnchorB;
                _referenceAngleB = prismatic.m_referenceAngle;
                _localAxisD      = prismatic.m_localXAxisA;

                var pD = _localAnchorD;
                var pB = MathUtils.MulT(
                    xfD.Rotation,
                    MathUtils.Mul(xfB.Rotation, _localAnchorB) + (xfB.Position - xfD.Position));
                coordinateB = MathUtils.Dot(pB - pD, _localAxisD);
            }

            _ratio = def.ratio;

            Constant = coordinateA + _ratio * coordinateB;

            _impulse = 0.0f;
        }

        /// Get the first joint.
        private Joint GetJoint1()
        {
            return _joint1;
        }

        /// Get the second joint.
        private Joint GetJoint2()
        {
            return _joint2;
        }

        /// Set/Get the gear ratio.
        private void SetRatio(float ratio)
        {
            Debug.Assert(ratio.IsValid());
            _ratio = ratio;
        }

        private float GetRatio()
        {
            return _ratio;
        }

        /// <inheritdoc />
        public override void Dump()
        {
            var indexA = BodyA._islandIndex;
            var indexB = BodyB._islandIndex;

            var index1 = _joint1.Index;
            var index2 = _joint2.Index;

            Logger.Log("  b2GearJointDef jd;");
            Logger.Log($"  jd.bodyA = bodies[{indexA}];");
            Logger.Log($"  jd.bodyB = bodies[{indexB}];");
            Logger.Log($"  jd.collideConnected = bool({CollideConnected});");
            Logger.Log("  jd.joint1 = joints[index1];");
            Logger.Log($"  jd.joint2 = joints[{index2}];");
            Logger.Log($"  jd.ratio = {_ratio};");
            Logger.Log($"  joints[{Index}] = m_world.CreateJoint(&jd);");
        }

        /// <inheritdoc />
        public override Vector2 GetAnchorA()
        {
            return BodyA.GetWorldPoint(_localAnchorA);
        }

        /// <inheritdoc />
        public override Vector2 GetAnchorB()
        {
            return BodyB.GetWorldPoint(_localAnchorB);
        }

        /// <inheritdoc />
        public override Vector2 GetReactionForce(float inv_dt)
        {
            var P = _impulse * _JvAC;
            return inv_dt * P;
        }

        /// <inheritdoc />
        public override float GetReactionTorque(float inv_dt)
        {
            var L = _impulse * _jwA;
            return inv_dt * L;
        }

        /// <inheritdoc />
        internal override void InitVelocityConstraints(SolverData data)
        {
            _indexA = BodyA._islandIndex;
            _indexB = BodyB._islandIndex;
            _indexC = BodyC._islandIndex;
            _indexD = BodyD._islandIndex;
            _lcA    = BodyA._sweep.localCenter;
            _lcB    = BodyB._sweep.localCenter;
            _lcC    = BodyC._sweep.localCenter;
            _lcD    = BodyD._sweep.localCenter;
            _mA     = BodyA._invMass;
            _mB     = BodyB._invMass;
            _mC     = BodyC._invMass;
            _mD     = BodyD._invMass;
            _iA     = BodyA._inverseInertia;
            _iB     = BodyB._inverseInertia;
            _iC     = BodyC._inverseInertia;
            _iD     = BodyD._inverseInertia;

            var aA = data.Positions[_indexA].Angle;
            var vA = data.Velocities[_indexA].v;
            var wA = data.Velocities[_indexA].w;

            var aB = data.Positions[_indexB].Angle;
            var vB = data.Velocities[_indexB].v;
            var wB = data.Velocities[_indexB].w;

            var aC = data.Positions[_indexC].Angle;
            var vC = data.Velocities[_indexC].v;
            var wC = data.Velocities[_indexC].w;

            var aD = data.Positions[_indexD].Angle;
            var vD = data.Velocities[_indexD].v;
            var wD = data.Velocities[_indexD].w;

            Rotation qA = new Rotation(aA), qB = new Rotation(aB), qC = new Rotation(aC), qD = new Rotation(aD);

            _mass = 0.0f;

            if (_typeA == JointType.RevoluteJoint)
            {
                _JvAC.SetZero();
                _jwA  =  1.0f;
                _jwC  =  1.0f;
                _mass += _iA + _iC;
            }
            else
            {
                var u  = MathUtils.Mul(qC, _localAxisC);
                var rC = MathUtils.Mul(qC, _localAnchorC - _lcC);
                var rA = MathUtils.Mul(qA, _localAnchorA - _lcA);
                _JvAC =  u;
                _jwC  =  MathUtils.Cross(rC, u);
                _jwA  =  MathUtils.Cross(rA, u);
                _mass += _mC + _mA + _iC * _jwC * _jwC + _iA * _jwA * _jwA;
            }

            if (_typeB == JointType.RevoluteJoint)
            {
                _JvBD.SetZero();
                _jwB  =  _ratio;
                _jwD  =  _ratio;
                _mass += _ratio * _ratio * (_iB + _iD);
            }
            else
            {
                var u  = MathUtils.Mul(qD, _localAxisD);
                var rD = MathUtils.Mul(qD, _localAnchorD - _lcD);
                var rB = MathUtils.Mul(qB, _localAnchorB - _lcB);
                _JvBD =  _ratio * u;
                _jwD  =  _ratio * MathUtils.Cross(rD, u);
                _jwB  =  _ratio * MathUtils.Cross(rB, u);
                _mass += _ratio * _ratio * (_mD + _mB) + _iD * _jwD * _jwD + _iB * _jwB * _jwB;
            }

            // Compute effective mass.
            _mass = _mass > 0.0f ? 1.0f / _mass : 0.0f;

            if (data.Step.warmStarting)
            {
                vA += _mA * _impulse * _JvAC;
                wA += _iA * _impulse * _jwA;
                vB += _mB * _impulse * _JvBD;
                wB += _iB * _impulse * _jwB;
                vC -= _mC * _impulse * _JvAC;
                wC -= _iC * _impulse * _jwC;
                vD -= _mD * _impulse * _JvBD;
                wD -= _iD * _impulse * _jwD;
            }
            else
            {
                _impulse = 0.0f;
            }

            data.Velocities[_indexA].v = vA;
            data.Velocities[_indexA].w = wA;
            data.Velocities[_indexB].v = vB;
            data.Velocities[_indexB].w = wB;
            data.Velocities[_indexC].v = vC;
            data.Velocities[_indexC].w = wC;
            data.Velocities[_indexD].v = vD;
            data.Velocities[_indexD].w = wD;
        }

        /// <inheritdoc />
        internal override void SolveVelocityConstraints(SolverData data)
        {
            var vA = data.Velocities[_indexA].v;
            var wA = data.Velocities[_indexA].w;
            var vB = data.Velocities[_indexB].v;
            var wB = data.Velocities[_indexB].w;
            var vC = data.Velocities[_indexC].v;
            var wC = data.Velocities[_indexC].w;
            var vD = data.Velocities[_indexD].v;
            var wD = data.Velocities[_indexD].w;

            var Cdot = MathUtils.Dot(_JvAC, vA - vC) + MathUtils.Dot(_JvBD, vB - vD);
            Cdot += _jwA * wA - _jwC * wC + (_jwB * wB - _jwD * wD);

            var impulse = -_mass * Cdot;
            _impulse += impulse;

            vA += _mA * impulse * _JvAC;
            wA += _iA * impulse * _jwA;
            vB += _mB * impulse * _JvBD;
            wB += _iB * impulse * _jwB;
            vC -= _mC * impulse * _JvAC;
            wC -= _iC * impulse * _jwC;
            vD -= _mD * impulse * _JvBD;
            wD -= _iD * impulse * _jwD;

            data.Velocities[_indexA].v = vA;
            data.Velocities[_indexA].w = wA;
            data.Velocities[_indexB].v = vB;
            data.Velocities[_indexB].w = wB;
            data.Velocities[_indexC].v = vC;
            data.Velocities[_indexC].w = wC;
            data.Velocities[_indexD].v = vD;
            data.Velocities[_indexD].w = wD;
        }

        /// <inheritdoc />
        internal override bool SolvePositionConstraints(SolverData data)
        {
            var cA = data.Positions[_indexA].Center;
            var aA = data.Positions[_indexA].Angle;
            var cB = data.Positions[_indexB].Center;
            var aB = data.Positions[_indexB].Angle;
            var cC = data.Positions[_indexC].Center;
            var aC = data.Positions[_indexC].Angle;
            var cD = data.Positions[_indexD].Center;
            var aD = data.Positions[_indexD].Angle;

            var qA = new Rotation(aA);
            var qB = new Rotation(aB);
            var qC = new Rotation(aC);
            var qD = new Rotation(aD);

            var linearError = 0.0f;

            float coordinateA, coordinateB;

            var   JvAC = new Vector2();
            var   JvBD = new Vector2();
            float JwA, JwB, JwC, JwD;
            var   mass = 0.0f;

            if (_typeA == JointType.RevoluteJoint)
            {
                JvAC.SetZero();
                JwA  =  1.0f;
                JwC  =  1.0f;
                mass += _iA + _iC;

                coordinateA = aA - aC - _referenceAngleA;
            }
            else
            {
                var u  = MathUtils.Mul(qC, _localAxisC);
                var rC = MathUtils.Mul(qC, _localAnchorC - _lcC);
                var rA = MathUtils.Mul(qA, _localAnchorA - _lcA);
                JvAC =  u;
                JwC  =  MathUtils.Cross(rC, u);
                JwA  =  MathUtils.Cross(rA, u);
                mass += _mC + _mA + _iC * JwC * JwC + _iA * JwA * JwA;

                var pC = _localAnchorC - _lcC;
                var pA = MathUtils.MulT(qC, rA + (cA - cC));
                coordinateA = MathUtils.Dot(pA - pC, _localAxisC);
            }

            if (_typeB == JointType.RevoluteJoint)
            {
                JvBD.SetZero();
                JwB  =  _ratio;
                JwD  =  _ratio;
                mass += _ratio * _ratio * (_iB + _iD);

                coordinateB = aB - aD - _referenceAngleB;
            }
            else
            {
                var u  = MathUtils.Mul(qD, _localAxisD);
                var rD = MathUtils.Mul(qD, _localAnchorD - _lcD);
                var rB = MathUtils.Mul(qB, _localAnchorB - _lcB);
                JvBD =  _ratio * u;
                JwD  =  _ratio * MathUtils.Cross(rD, u);
                JwB  =  _ratio * MathUtils.Cross(rB, u);
                mass += _ratio * _ratio * (_mD + _mB) + _iD * JwD * JwD + _iB * JwB * JwB;

                var pD = _localAnchorD - _lcD;
                var pB = MathUtils.MulT(qD, rB + (cB - cD));
                coordinateB = MathUtils.Dot(pB - pD, _localAxisD);
            }

            var C = coordinateA + _ratio * coordinateB - Constant;

            var impulse = 0.0f;
            if (mass > 0.0f)
            {
                impulse = -C / mass;
            }

            cA += _mA * impulse * JvAC;
            aA += _iA * impulse * JwA;
            cB += _mB * impulse * JvBD;
            aB += _iB * impulse * JwB;
            cC -= _mC * impulse * JvAC;
            aC -= _iC * impulse * JwC;
            cD -= _mD * impulse * JvBD;
            aD -= _iD * impulse * JwD;

            data.Positions[_indexA].Center = cA;
            data.Positions[_indexA].Angle = aA;
            data.Positions[_indexB].Center = cB;
            data.Positions[_indexB].Angle = aB;
            data.Positions[_indexC].Center = cC;
            data.Positions[_indexC].Angle = aC;
            data.Positions[_indexD].Center = cD;
            data.Positions[_indexD].Angle = aD;

            // TODO_ERIN not implemented
            return linearError < Settings.LinearSlop;
        }
    }
}