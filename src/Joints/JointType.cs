namespace Box2DSharp
{
    /// Joint type enumeration
    ///
    /// This is useful because all joint types use b2JointId and sometimes you
    /// want to get the type of a joint.
    /// @ingroup joint
    public enum JointType
    {
        DistanceJoint,

        MotorJoint,

        MouseJoint,

        PrismaticJoint,

        RevoluteJoint,

        WeldJoint,

        WheelJoint
    }
}