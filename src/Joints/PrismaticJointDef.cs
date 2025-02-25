namespace Box2DSharp
{
    /// Prismatic joint definition
    ///
    /// This requires defining a line of motion using an axis and an anchor point.
    /// The definition uses local anchor points and a local axis so that the initial
    /// configuration can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space.
    /// @ingroup prismatic_joint
    public struct PrismaticJointDef
    {
        /// The first attached body
        public BodyId BodyIdA;

        /// The second attached body
        public BodyId BodyIdB;

        /// The local anchor point relative to bodyA's origin
        public Vec2 LocalAnchorA;

        /// The local anchor point relative to bodyB's origin
        public Vec2 LocalAnchorB;

        /// The local translation unit axis in bodyA
        public Vec2 LocalAxisA;

        /// The constrained angle between the bodies: bodyB_angle - bodyA_angle
        public float ReferenceAngle;

        /// Enable a linear spring along the prismatic joint axis
        public bool EnableSpring;

        /// The spring stiffness Hertz, cycles per second
        public float Hertz;

        /// The spring damping ratio, non-dimensional
        public float DampingRatio;

        /// Enable/disable the joint limit
        public bool EnableLimit;

        /// The lower translation limit
        public float LowerTranslation;

        /// The upper translation limit
        public float UpperTranslation;

        /// Enable/disable the joint motor
        public bool EnableMotor;

        /// The maximum motor force, typically in newtons
        public float MaxMotorForce;

        /// The desired motor speed, typically in meters per second
        public float MotorSpeed;

        /// Set this flag to true if the attached bodies should collide
        public bool CollideConnected;

        /// User data pointer
        public object UserData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int InternalValue;

        /// Use this to initialize your joint definition
        /// @ingroupd prismatic_joint
        public static PrismaticJointDef DefaultPrismaticJointDef()
        {
            return new PrismaticJointDef
            {
                LocalAxisA = Vec2.UnitX,
                InternalValue = Core.SecretCookie
            };
        }
    }
}