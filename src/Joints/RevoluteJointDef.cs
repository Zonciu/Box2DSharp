namespace Box2DSharp
{
    /// Revolute joint definition
    ///
    /// This requires defining an anchor point where the bodies are joined.
    /// The definition uses local anchor points so that the
    /// initial configuration can violate the constraint slightly. You also need to
    /// specify the initial relative angle for joint limits. This helps when saving
    /// and loading a game.
    /// The local anchor points are measured from the body's origin
    /// rather than the center of mass because:
    /// 1. you might not know where the center of mass will be
    /// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken
    /// @ingroup revolute_joint
    public struct RevoluteJointDef
    {
        /// The first attached body
        public BodyId BodyIdA;

        /// The second attached body
        public BodyId BodyIdB;

        /// The local anchor point relative to bodyA's origin
        public Vec2 LocalAnchorA;

        /// The local anchor point relative to bodyB's origin
        public Vec2 LocalAnchorB;

        /// The bodyB angle minus bodyA angle in the reference state (radians).
        /// This defines the zero angle for the joint limit.
        public float ReferenceAngle;

        /// Enable a rotational spring on the revolute hinge axis
        public bool EnableSpring;

        /// The spring stiffness Hertz, cycles per second
        public float Hertz;

        /// The spring damping ratio, non-dimensional
        public float DampingRatio;

        /// A flag to enable joint limits
        public bool EnableLimit;

        /// The lower angle for the joint limit in radians
        public float LowerAngle;

        /// The upper angle for the joint limit in radians
        public float UpperAngle;

        /// A flag to enable the joint motor
        public bool EnableMotor;

        /// The maximum motor torque, typically in newton-meters
        public float MaxMotorTorque;

        /// The desired motor speed in radians per second
        public float MotorSpeed;

        /// Scale the debug draw
        public float DrawSize;

        /// Set this flag to true if the attached bodies should collide
        public bool CollideConnected;

        /// User data pointer
        public object? UserData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int InternalValue;

        public RevoluteJointDef(float hertz, float lowerAngle)
        {
            Hertz = hertz;
            LowerAngle = lowerAngle;
            BodyIdA = BodyId.NullId;
            BodyIdB = BodyId.NullId;
            LocalAnchorA = default;
            LocalAnchorB = default;
            ReferenceAngle = 0;
            EnableSpring = false;
            DampingRatio = 0;
            EnableLimit = false;
            UpperAngle = 0;
            EnableMotor = false;
            MaxMotorTorque = 0;
            MotorSpeed = 0;
            DrawSize = 0;
            CollideConnected = false;
            UserData = null;
            InternalValue = 0;
        }

        /// Use this to initialize your joint definition.
        /// @ingroup revolute_joint
        public static RevoluteJointDef DefaultRevoluteJointDef()
        {
            return new RevoluteJointDef
            {
                DrawSize = 0.25f,
                InternalValue = Core.SecretCookie
            };
        }
    }
}