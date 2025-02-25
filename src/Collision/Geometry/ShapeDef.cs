namespace Box2DSharp
{
    /// Used to create a shape.
    /// This is a temporary object used to bundle shape creation parameters. You may use
    ///	the same shape definition to create multiple shapes.
    /// Must be initialized using b2DefaultShapeDef().
    /// @ingroup shape
    public struct ShapeDef
    {
        /// Use this to store application specific shape data.
        public object UserData;

        /// The Coulomb (dry) friction coefficient, usually in the range [0,1].
        public float Friction;

        /// The restitution (bounce) usually in the range [0,1].
        public float Restitution;

        /// The density, usually in kg/m^2.
        public float Density;

        /// Collision filtering data.
        public Filter Filter;

        /// Custom debug draw color.
        public uint CustomColor;

        /// A sensor shape generates overlap events but never generates a collision response.
        ///	Sensors do not collide with other sensors and do not have continuous collision.
        ///	Instead use a ray or shape cast for those scenarios.
        public bool IsSensor;

        /// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        public bool EnableSensorEvents;

        /// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        public bool EnableContactEvents;

        /// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        public bool EnableHitEvents;

        /// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
        ///	and must be carefully handled due to threading. Ignored for sensors.
        public bool EnablePreSolveEvents;

        /// Normally shapes on static bodies don't invoke contact creation when they are added to the world. This overrides
        ///	that behavior and causes contact creation. This significantly slows down static body creation which can be important
        ///	when there are many static shapes.
        /// This is implicitly always true for sensors.
        public bool ForceContactCreation;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int InternalValue;

        /// Use this to initialize your shape definition
        /// @ingroup shape
        public static ShapeDef DefaultShapeDef()
        {
            return new ShapeDef
            {
                Friction = 0.6f,
                Density = 1.0f,
                Filter = Filter.DefaultFilter(),
                EnableSensorEvents = true,
                EnableContactEvents = true,
                InternalValue = Core.SecretCookie
            };
        }
    }
}