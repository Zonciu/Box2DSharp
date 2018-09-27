using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// Friction joint definition.
    public class FrictionJointDef : JointDef
    {
        public FrictionJointDef()
        {
            JointType = JointType.FrictionJoint;
            localAnchorA.SetZero();
            localAnchorB.SetZero();
            maxForce  = 0.0f;
            maxTorque = 0.0f;
        }

        // Point-to-point constraint
        // Cdot = v2 - v1
        //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
        // J = [-I -r1_skew I r2_skew ]
        // Identity used:
        // w k % (rx i + ry j) = w * (-ry i + rx j)

        // Angle constraint
        // Cdot = w2 - w1
        // J = [0 0 -1 0 0 1]
        // K = invI1 + invI2
        /// Initialize the bodies, anchors, axis, and reference angle using the world
        /// anchor and world axis.
        public void Initialize(Body bA, Body bB, in Vector2 anchor)
        {
            BodyA        = bA;
            BodyB        = bB;
            localAnchorA = BodyA.GetLocalPoint(anchor);
            localAnchorB = BodyB.GetLocalPoint(anchor);
        }

        /// The local anchor point relative to bodyA's origin.
        public Vector2 localAnchorA;

        /// The local anchor point relative to bodyB's origin.
        public Vector2 localAnchorB;

        /// The maximum friction force in N.
        public float maxForce;

        /// The maximum friction torque in N-m.
        public float maxTorque;
    };
}