using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// Weld joint definition. You need to specify local anchor points
    /// where they are attached and the relative body angle. The position
    /// of the anchor points is important for computing the reaction torque.
    public class WeldJointDef : JointDef
    {
        public WeldJointDef()
        {
            JointType = JointType.WeldJoint;
            localAnchorA.Set(0.0f, 0.0f);
            localAnchorB.Set(0.0f, 0.0f);
            referenceAngle = 0.0f;
            frequencyHz    = 0.0f;
            dampingRatio   = 0.0f;
        }

        /// Initialize the bodies, anchors, and reference angle using a world
        /// anchor point.
        internal void Initialize(Body bA, Body bB, in Vector2 anchor)
        {
            BodyA          = bA;
            BodyB          = bB;
            localAnchorA   = BodyA.GetLocalPoint(anchor);
            localAnchorB   = BodyB.GetLocalPoint(anchor);
            referenceAngle = BodyB.GetAngle() - BodyA.GetAngle();
        }

        /// The local anchor point relative to bodyA's origin.
        public Vector2 localAnchorA;

        /// The local anchor point relative to bodyB's origin.
        public Vector2 localAnchorB;

        /// The bodyB angle minus bodyA angle in the reference state (radians).
        public float referenceAngle;

        /// The mass-spring-damper frequency in Hertz. Rotation only.
        /// Disable softness with a value of 0.
        public float frequencyHz;

        /// The damping ratio. 0 = no damping, 1 = critical damping.
        public float dampingRatio;
    };
}