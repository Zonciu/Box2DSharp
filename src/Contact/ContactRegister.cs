namespace Box2DSharp
{
    public class ContactRegister
    {
        public ManifoldFcn Fcn;

        public bool Primary;

        public static readonly ContactRegister[,] Registers = new ContactRegister[(int)ShapeType.ShapeTypeCount, (int)ShapeType.ShapeTypeCount];

        public static bool Initialized = false;

        public ContactRegister(ManifoldFcn fcn, bool primary)
        {
            Fcn = fcn;
            Primary = primary;
        }
    }
}