namespace Box2DSharp
{
    /// Weld joint definition
    ///
    /// A weld joint connect to bodies together rigidly. This constraint provides springs to mimic
    ///	soft-body simulation.
    /// @note The approximate solver in Box2D cannot hold many bodies together rigidly
    /// @ingroup weld_joint
    public struct WeldJointDef
    {
        /// The first attached body
        public BodyId BodyIdA;

        /// The second attached body
        public BodyId BodyIdB;

        /// The local anchor point relative to bodyA's origin
        public Vec2 LocalAnchorA;

        /// The local anchor point relative to bodyB's origin
        public Vec2 LocalAnchorB;

        /// The bodyB angle minus bodyA angle in the reference state (radians)
        public float ReferenceAngle;

        /// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
        public float LinearHertz;

        /// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
        public float AngularHertz;

        /// Linear damping ratio, non-dimensional. Use 1 for critical damping.
        public float LinearDampingRatio;

        /// Linear damping ratio, non-dimensional. Use 1 for critical damping.
        public float AngularDampingRatio;

        /// Set this flag to true if the attached bodies should collide
        public bool CollideConnected;

        /// User data pointer
        public object UserData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int InternalValue;

        /// Use this to initialize your joint definition
        /// @ingroup weld_joint
        public static WeldJointDef DefaultWeldJointDef()
        {
            return new WeldJointDef
            {
                InternalValue = Core.SecretCookie
            };
        }
    }
}