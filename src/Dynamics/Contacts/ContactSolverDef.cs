namespace Box2DSharp.Dynamics.Contacts
{
    public class ContactSolverDef
    {
        public TimeStep step;

        public Contact[] contacts;

        public int count;

        public Position[] positions;

        public Velocity[] velocities;
    };
}