namespace Box2DSharp
{
    /// Distance joint definition
    ///
    /// This requires defining an anchor point on both
    /// bodies and the non-zero distance of the distance joint. The definition uses
    /// local anchor points so that the initial configuration can violate the
    /// constraint slightly. This helps when saving and loading a game.
    /// @ingroup distance_joint
    public struct DistanceJointDef
    {
        /// The first attached body
        public BodyId BodyIdA;

        /// The second attached body
        public BodyId BodyIdB;

        /// The local anchor point relative to bodyA's origin
        public Vec2 LocalAnchorA;

        /// The local anchor point relative to bodyB's origin
        public Vec2 LocalAnchorB;

        /// The rest length of this joint. Clamped to a stable minimum value.
        public float Length;

        /// Enable the distance constraint to behave like a spring. If false
        ///	then the distance joint will be rigid, overriding the limit and motor.
        public bool EnableSpring;

        /// The spring linear stiffness Hertz, cycles per second
        public float Hertz;

        /// The spring linear damping ratio, non-dimensional
        public float DampingRatio;

        /// Enable/disable the joint limit
        public bool EnableLimit;

        /// Minimum length. Clamped to a stable minimum value.
        public float MinLength;

        /// Maximum length. Must be greater than or equal to the minimum length.
        public float MaxLength;

        /// Enable/disable the joint motor
        public bool EnableMotor;

        /// The maximum motor force, usually in newtons
        public float MaxMotorForce;

        /// The desired motor speed, usually in meters per second
        public float MotorSpeed;

        /// Set this flag to true if the attached bodies should collide
        public bool CollideConnected;

        /// User data pointer
        public object UserData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int InternalValue;

        /// Use this to initialize your joint definition
        /// @ingroup distance_joint
        public static DistanceJointDef DefaultDistanceJointDef()
        {
            return new DistanceJointDef
            {
                Length = 1.0f,
                MaxLength = Core.Huge,
                InternalValue = Core.SecretCookie
            };
        }
    }
}