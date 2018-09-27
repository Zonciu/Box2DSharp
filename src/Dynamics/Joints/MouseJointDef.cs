using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// Mouse joint definition. This requires a world target point,
    /// tuning parameters, and the time step.
    public class MouseJointDef : JointDef
    {
        public MouseJointDef()
        {
            JointType = JointType.MouseJoint;
            target.Set(0.0f, 0.0f);
            maxForce     = 0.0f;
            frequencyHz  = 5.0f;
            dampingRatio = 0.7f;
        }

        /// The initial world target point. This is assumed
        /// to coincide with the body anchor initially.
        public Vector2 target;

        /// The maximum constraint force that can be exerted
        /// to move the candidate body. Usually you will express
        /// as some multiple of the weight (multiplier * mass * gravity).
        public float maxForce;

        /// The response speed.
        public float frequencyHz;

        /// The damping ratio. 0 = no damping, 1 = critical damping.
        public float dampingRatio;
    };
}