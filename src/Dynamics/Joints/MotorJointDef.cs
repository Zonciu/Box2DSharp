using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// Motor joint definition.
    public class MotorJointDef : JointDef
    {
        public MotorJointDef()
        {
            JointType = JointType.MotorJoint;
            linearOffset.SetZero();
            angularOffset    = 0.0f;
            maxForce         = 1.0f;
            maxTorque        = 1.0f;
            correctionFactor = 0.3f;
        }

        /// Initialize the bodies and offsets using the current transforms.
        internal void Initialize(Body bA, Body bB)
        {
            BodyA = bA;
            BodyB = bB;
            var xB = BodyB.GetPosition();
            linearOffset = BodyA.GetLocalPoint(xB);

            var angleA = BodyA.GetAngle();
            var angleB = BodyB.GetAngle();
            angularOffset = angleB - angleA;
        }

        /// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
        public Vector2 linearOffset;

        /// The bodyB angle minus bodyA angle in radians.
        public float angularOffset;

        /// The maximum motor force in N.
        public float maxForce;

        /// The maximum motor torque in N-m.
        public float maxTorque;

        /// Position correction factor in the range [0,1].
        public float correctionFactor;
    };
}