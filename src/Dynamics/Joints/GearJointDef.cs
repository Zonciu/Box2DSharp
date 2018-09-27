namespace Box2DSharp.Dynamics.Joints
{
    /// Gear joint definition. This definition requires two existing
    /// revolute or prismatic joints (any combination will work).
    public class GearJointDef : JointDef
    {
        /// The first revolute/prismatic joint attached to the gear joint.
        public Joint joint1;

        /// The second revolute/prismatic joint attached to the gear joint.
        public Joint joint2;

        /// The gear ratio.
        /// @see b2GearJoint for explanation.
        public float ratio;

        private GearJointDef()
        {
            JointType = JointType.GearJoint;
            joint1    = null;
            joint2    = null;
            ratio     = 1.0f;
        }
    }
}