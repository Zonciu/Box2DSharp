namespace Box2DSharp
{
    /// A body definition holds all the data needed to construct a rigid body.
    /// You can safely re-use body definitions. Shapes are added to a body after construction.
    ///	Body definitions are temporary objects used to bundle creation parameters.
    /// Must be initialized using b2DefaultBodyDef().
    /// @ingroup body
    public struct BodyDef
    {
        /// The body type: static, kinematic, or dynamic.
        public BodyType Type;

        /// The initial world position of the body. Bodies should be created with the desired position.
        /// @note Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
        ///	if the body is moved after shapes have been added.
        public Vec2 Position;

        /// The initial world rotation of the body. Use b2MakeRot() if you have an angle.
        public Rot Rotation;

        /// The initial linear velocity of the body's origin. Typically in meters per second.
        public Vec2 LinearVelocity;

        /// The initial angular velocity of the body. Radians per second.
        public float AngularVelocity;

        /// Linear damping is use to reduce the linear velocity. The damping parameter
        /// can be larger than 1 but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        ///	Generally linear damping is undesirable because it makes objects move slowly
        ///	as if they are floating.
        public float LinearDamping;

        /// Angular damping is use to reduce the angular velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        ///	Angular damping can be use slow down rotating bodies.
        public float AngularDamping;

        /// Scale the gravity applied to this body. Non-dimensional.
        public float GravityScale;

        /// Sleep velocity threshold, default is 0.05 meter per second
        public float SleepThreshold;

        /// Use this to store application specific body data.
        public object UserData;

        /// Set this flag to false if this body should never fall asleep.
        public bool EnableSleep;

        /// Is this body initially awake or sleeping?
        public bool IsAwake;

        /// Should this body be prevented from rotating? Useful for characters.
        public bool FixedRotation;

        /// Treat this body as high speed object that performs continuous collision detection
        /// against dynamic and kinematic bodies, but not other bullet bodies.
        ///	@warning Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
        ///	continuous collision. They may interfere with joint constraints.
        public bool IsBullet;

        /// Used to disable a body. A disabled body does not move or collide.
        public bool IsEnabled;

        /// Automatically compute mass and related properties on this body from shapes.
        /// Triggers whenever a shape is add/removed/changed. Default is true.
        public bool AutomaticMass;

        /// This allows this body to bypass rotational speed limits. Should only be used
        ///	for circular objects, like wheels.
        public bool AllowFastRotation;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int InternalValue;

        /// Use this to initialize your body definition
        /// @ingroup body
        public static BodyDef DefaultBodyDef()
        {
            return new BodyDef
            {
                Type = BodyType.StaticBody,
                Rotation = Rot.Identity,
                SleepThreshold = 0.05f * Core.LengthUnitsPerMeter,
                GravityScale = 1.0f,
                EnableSleep = true,
                IsAwake = true,
                IsEnabled = true,
                AutomaticMass = true,
                InternalValue = Core.SecretCookie
            };
        }
    }
}