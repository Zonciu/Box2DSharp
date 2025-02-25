namespace Box2DSharp
{
    /// A motor joint is used to control the relative motion between two bodies
    ///
    /// A typical usage is to control the movement of a dynamic body with respect to the ground.
    /// @ingroup motor_joint
    public struct MotorJointDef
    {
        /// The first attached body
        public BodyId BodyIdA;

        /// The second attached body
        public BodyId BodyIdB;

        /// Position of bodyB minus the position of bodyA, in bodyA's frame
        public Vec2 LinearOffset;

        /// The bodyB angle minus bodyA angle in radians
        public float AngularOffset;

        /// The maximum motor force in newtons
        public float MaxForce;

        /// The maximum motor torque in newton-meters
        public float MaxTorque;

        /// Position correction factor in the range [0,1]
        public float CorrectionFactor;

        /// Set this flag to true if the attached bodies should collide
        public bool CollideConnected;

        /// User data pointer
        public object UserData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int InternalValue;

        /// Use this to initialize your joint definition
        /// @ingroup motor_joint
        public static MotorJointDef DefaultMotorJointDef()
        {
            return new MotorJointDef
            {
                MaxForce = 1.0f,
                MaxTorque = 1.0f,
                CorrectionFactor = 0.3f,
                InternalValue = Core.SecretCookie
            };
        }
    }
}