namespace Box2DSharp
{
    /// Wheel joint definition
    ///
    /// This requires defining a line of motion using an axis and an anchor point.
    /// The definition uses local  anchor points and a local axis so that the initial
    /// configuration can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space.
    /// @ingroup wheel_joint
    public struct WheelJointDef
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

        /// Enable a linear spring along the local axis
        public bool EnableSpring;

        /// Spring stiffness in Hertz
        public float Hertz;

        /// Spring damping ratio, non-dimensional
        public float DampingRatio;

        /// Enable/disable the joint linear limit
        public bool EnableLimit;

        /// The lower translation limit
        public float LowerTranslation;

        /// The upper translation limit
        public float UpperTranslation;

        /// Enable/disable the joint rotational motor
        public bool EnableMotor;

        /// The maximum motor torque, typically in newton-meters
        public float MaxMotorTorque;

        /// The desired motor speed in radians per second
        public float MotorSpeed;

        /// Set this flag to true if the attached bodies should collide
        public bool CollideConnected;

        /// User data pointer
        public object UserData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int InternalValue;

        /// Use this to initialize your joint definition
        /// @ingroup wheel_joint
        public static WheelJointDef DefaultWheelJointDef()
        {
            return new WheelJointDef
            {
                LocalAxisA = Vec2.UnitY,
                EnableSpring = true,
                Hertz = 1.0f,
                DampingRatio = 0.7f,
                InternalValue = Core.SecretCookie
            };
        }
    }
}