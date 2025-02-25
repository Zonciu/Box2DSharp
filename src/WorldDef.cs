namespace Box2DSharp
{
    /// <summary>
    /// World definition used to create a simulation world.
    /// Must be initialized using b2DefaultWorldDef().
    /// @ingroup world
    /// </summary>
    public struct WorldDef
    {
        /// Gravity vector. Box2D has no up-vector defined.
        public Vec2 Gravity;

        /// Restitution velocity threshold, usually in m/s. Collisions above this
        /// speed have restitution applied (will bounce).
        public float RestitutionThreshold;

        /// This parameter controls how fast overlap is resolved and has units of meters per second
        public float ContactPushoutVelocity;

        /// Threshold velocity for hit events. Usually meters per second.
        public float HitEventThreshold;

        /// Contact stiffness. Cycles per second.
        public float ContactHertz;

        /// Contact bounciness. Non-dimensional.
        public float ContactDampingRatio;

        /// Joint stiffness. Cycles per second.
        public float JointHertz;

        /// Joint bounciness. Non-dimensional.
        public float JointDampingRatio;

        /// Maximum linear velocity. Usually meters per second.
        public float MaximumLinearVelocity;

        /// Can bodies go to sleep to improve performance
        public bool EnableSleep;

        /// Enable continuous collision
        public bool EnableContinuous;

        /// Number of workers to use with the provided task system. Box2D performs best when using only
        ///	performance cores and accessing a single L2 cache. Efficiency cores and hyper-threading provide
        ///	little benefit and may even harm performance.
        public int WorkerCount;

        /// Function to spawn tasks
        public EnqueueTaskCallback? EnqueueTask;

        /// Function to finish a task
        public FinishTaskCallback? FinishTask;

        /// User context that is provided to enqueueTask and finishTask
        public object UserTaskContext;

        /// Used internally to detect a valid definition. DO NOT SET.
        public int InternalValue;

        /// Use this to initialize your world definition
        /// @ingroup world
        public static WorldDef DefaultWorldDef()
        {
            WorldDef def = new();
            def.Gravity.X = 0.0f;
            def.Gravity.Y = -10.0f;
            def.HitEventThreshold = 1.0f * Core.LengthUnitsPerMeter;
            def.RestitutionThreshold = 1.0f * Core.LengthUnitsPerMeter;
            def.ContactPushoutVelocity = 3.0f * Core.LengthUnitsPerMeter;
            def.ContactHertz = 30.0f;
            def.ContactDampingRatio = 10.0f;
            def.JointHertz = 60.0f;
            def.JointDampingRatio = 2.0f;

            // 400 meters per second, faster than the speed of sound
            def.MaximumLinearVelocity = 400.0f * Core.LengthUnitsPerMeter;
            def.EnableSleep = true;
            def.EnableContinuous = true;
            def.InternalValue = Core.SecretCookie;
            def.WorkerCount = 1;
            return def;
        }
    }
}