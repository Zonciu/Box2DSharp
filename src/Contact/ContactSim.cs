namespace Box2DSharp
{
    /// The class manages contact between two shapes. A contact exists for each overlapping
    /// AABB in the broad-phase (except if filtered). Therefore, a contact object may exist
    /// that has no contact points.
    /// 该类管理两个形状之间的接触。每个重叠的粗检测中的AABB（过滤除外）都会存在一个接触点对象。
    /// 因此，两个形状之间没有接触点也可能存在接触对象。
    public class ContactSim
    {
        public int ContactId = Core.NullIndex;

        public int BodyIdA = Core.NullIndex;

        public int BodyIdB = Core.NullIndex;

        public int BodySimIndexA = Core.NullIndex;

        public int BodySimIndexB = Core.NullIndex;

        public int ShapeIdA = Core.NullIndex;

        public int ShapeIdB = Core.NullIndex;

        public float InvMassA;

        public float InvIA;

        public float InvMassB;

        public float InvIB;

        public Manifold Manifold;

        /// <summary>
        /// Mixed friction and restitution
        /// </summary>
        public float Friction;

        public float Restitution;

        /// <summary>
        /// todo for conveyor belts
        /// </summary>
        public float TangentSpeed;

        // b2ContactSimFlags
        public ContactSimFlags SimFlags;

        public DistanceCache Cache;

        public override string ToString()
        {
            return $"{ContactId} {BodyIdA} {BodyIdB}";
        }

        public void CopyTo(ContactSim v)
        {
            v.ContactId = ContactId;
            v.BodyIdA = BodyIdA;
            v.BodyIdB = BodyIdB;
            v.BodySimIndexA = BodySimIndexA;
            v.BodySimIndexB = BodySimIndexB;
            v.ShapeIdA = ShapeIdA;
            v.ShapeIdB = ShapeIdB;
            v.InvMassA = InvMassA;
            v.InvIA = InvIA;
            v.InvMassB = InvMassB;
            v.InvIB = InvIB;
            v.Manifold = Manifold;
            v.Friction = Friction;
            v.Restitution = Restitution;
            v.TangentSpeed = TangentSpeed;
            v.SimFlags = SimFlags;
            v.Cache = Cache;
        }

        public void Reset()
        {
            ContactId = Core.NullIndex;
            BodyIdA = Core.NullIndex;
            BodyIdB = Core.NullIndex;
            BodySimIndexA = Core.NullIndex;
            BodySimIndexB = Core.NullIndex;
            ShapeIdA = Core.NullIndex;
            ShapeIdB = Core.NullIndex;
            InvMassA = default;
            InvIA = default;
            InvMassB = default;
            InvIB = default;
            Manifold = default;
            Friction = default;
            Restitution = default;
            TangentSpeed = default;
            SimFlags = default;
            Cache = default;
        }
    }
}