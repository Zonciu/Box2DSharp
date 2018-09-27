using System.Numerics;

namespace Box2DSharp.Dynamics
{
    /// Profiling data. Times are in milliseconds.
    public struct Profile
    {
        public float step;

        public float collide;

        public float solve;

        public float solveInit;

        public float solveVelocity;

        public float solvePosition;

        public float broadphase;

        public float solveTOI;
    };

    /// This is an internal structure.
    public struct TimeStep
    {
        public float dt; // time step

        public float inv_dt; // inverse time step (0 if dt == 0).

        public float dtRatio; // dt * inv_dt0

        public int velocityIterations;

        public int positionIterations;

        public bool warmStarting;
    };

    /// This is an internal structure.
    public struct Position
    {
        public Vector2 Center;

        public float Angle;
    };

    /// This is an internal structure.
    public struct Velocity
    {
        public Vector2 v;

        public float w;
    };

    /// Solver Data
    public class SolverData
    {
        public TimeStep Step;

        public Position[] Positions;

        public Velocity[] Velocities;
    };
}