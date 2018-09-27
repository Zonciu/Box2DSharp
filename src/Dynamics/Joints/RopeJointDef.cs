using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// Rope joint definition. This requires two body anchor points and
    /// a maximum lengths.
    /// Note: by default the connected objects will not collide.
    /// see collideConnected in b2JointDef.
    public class RopeJointDef : JointDef
    {
        RopeJointDef()
        {
            JointType = JointType.RopeJoint;
            localAnchorA.Set(-1.0f, 0.0f);
            localAnchorB.Set(1.0f, 0.0f);
            maxLength = 0.0f;
        }

        /// The local anchor point relative to bodyA's origin.
        public Vector2 localAnchorA;

        /// The local anchor point relative to bodyB's origin.
        public Vector2 localAnchorB;

        /// The maximum length of the rope.
        /// Warning: this must be larger than b2_linearSlop or
        /// the joint will have no effect.
        public float maxLength;
    };
}