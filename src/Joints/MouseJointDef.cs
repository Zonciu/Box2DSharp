namespace Box2DSharp
{
    /// A mouse joint is used to make a point on a body track a specified world point.
    ///
    /// This a soft constraint and allows the constraint to stretch without
    /// applying huge forces. This also applies rotation constraint heuristic to improve control.
    /// @ingroup mouse_joint
    public struct MouseJointDef
    {
        /// The first attached body.
        public BodyId BodyIdA;

        /// The second attached body.
        public BodyId BodyIdB;

        /// The initial target point in world space
        public Vec2 Target;

        /// Stiffness in hertz
        public float Hertz;

        /// Damping ratio, non-dimensional
        public float DampingRatio;

        /// Maximum force, typically in newtons
        public float MaxForce;

        /// Set this flag to true if the attached bodies should collide.
        public bool CollideConnected;

        /// User data pointer
        public object UserData;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int InternalValue;

        /// Use this to initialize your joint definition
        /// @ingroup mouse_joint
        public static MouseJointDef DefaultMouseJointDef()
        {
            return new MouseJointDef
            {
                Hertz = 4.0f,
                DampingRatio = 1.0f,
                MaxForce = 1.0f,
                InternalValue = Core.SecretCookie
            };
        }
    }
}