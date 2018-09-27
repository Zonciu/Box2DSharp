using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Joints
{
    /// Pulley joint definition. This requires two ground anchors,
    /// two dynamic body anchor points, and a pulley ratio.
    public class PulleyJointDef : JointDef
    {
        public PulleyJointDef()
        {
            JointType = JointType.PulleyJoint;

            groundAnchorA.Set(-1.0f, 1.0f);

            groundAnchorB.Set(1.0f, 1.0f);

            localAnchorA.Set(-1.0f, 0.0f);

            localAnchorB.Set(1.0f, 0.0f);

            lengthA = 0.0f;

            lengthB = 0.0f;

            ratio = 1.0f;

            CollideConnected = true;
        }

        /// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
        void Initialize(
            Body       bA,
            Body       bB,
            in Vector2 groundA,
            in Vector2 groundB,
            in Vector2 anchorA,
            in Vector2 anchorB,
            float      r)
        {
            BodyA         = bA;
            BodyB         = bB;
            groundAnchorA = groundA;
            groundAnchorB = groundB;
            localAnchorA  = BodyA.GetLocalPoint(anchorA);
            localAnchorB  = BodyB.GetLocalPoint(anchorB);
            var dA = anchorA - groundA;
            lengthA = dA.Length();
            var dB = anchorB - groundB;
            lengthB = dB.Length();
            ratio   = r;
            Debug.Assert(ratio > Settings.Epsilon);
        }

        /// The first ground anchor in world coordinates. This point never moves.
        public Vector2 groundAnchorA;

        /// The second ground anchor in world coordinates. This point never moves.
        public Vector2 groundAnchorB;

        /// The local anchor point relative to bodyA's origin.
        public Vector2 localAnchorA;

        /// The local anchor point relative to bodyB's origin.
        public Vector2 localAnchorB;

        /// The a reference length for the segment attached to bodyA.
        public float lengthA;

        /// The a reference length for the segment attached to bodyB.
        public float lengthB;

        /// The pulley ratio, used to simulate a block-and-tackle.
        public float ratio;
    };
}