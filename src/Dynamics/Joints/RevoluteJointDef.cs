using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// Revolute joint definition. This requires defining an
    /// anchor point where the bodies are joined. The definition
    /// uses local anchor points so that the initial configuration
    /// can violate the constraint slightly. You also need to
    /// specify the initial relative angle for joint limits. This
    /// helps when saving and loading a game.
    /// The local anchor points are measured from the body's origin
    /// rather than the center of mass because:
    /// 1. you might not know where the center of mass will be.
    /// 2. if you add/remove shapes from a body and recompute the mass,
    ///    the joints will be broken.
    public class RevoluteJointDef : JointDef
    {
        public RevoluteJointDef()
        {
            JointType = JointType.RevoluteJoint;
            localAnchorA.Set(0.0f, 0.0f);
            localAnchorB.Set(0.0f, 0.0f);
            referenceAngle = 0.0f;
            lowerAngle     = 0.0f;
            upperAngle     = 0.0f;
            maxMotorTorque = 0.0f;
            motorSpeed     = 0.0f;
            enableLimit    = false;
            enableMotor    = false;
        }

        /// Initialize the bodies, anchors, and reference angle using a world
        /// anchor point.
        internal void Initialize(Body bA, Body bB, Vector2 anchor)
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

        /// A flag to enable joint limits.
        public bool enableLimit;

        /// The lower angle for the joint limit (radians).
        public float lowerAngle;

        /// The upper angle for the joint limit (radians).
        public float upperAngle;

        /// A flag to enable the joint motor.
        public bool enableMotor;

        /// The desired motor speed. Usually in radians per second.
        public float motorSpeed;

        /// The maximum motor torque used to achieve the desired motor speed.
        /// Usually in N-m.
        public float maxMotorTorque;
    };
}