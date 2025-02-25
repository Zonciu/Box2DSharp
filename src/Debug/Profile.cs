namespace Box2DSharp
{
    /// <summary>
    /// Profiling data. Times are in milliseconds.
    /// </summary>
    public class Profile
    {
        public float Step;

        public float Pairs;

        public float Collide;

        public float Solve;

        public float BuildIslands;

        public float SolveConstraints;

        public float PrepareTasks;

        public float SolverTasks;

        public float PrepareConstraints;

        public float IntegrateVelocities;

        public float WarmStart;

        public float SolveVelocities;

        public float IntegratePositions;

        public float RelaxVelocities;

        public float ApplyRestitution;

        public float StoreImpulses;

        public float FinalizeBodies;

        public float SplitIslands;

        public float SleepIslands;

        public float HitEvents;

        public float Broadphase;

        public float Continuous;
    }
}