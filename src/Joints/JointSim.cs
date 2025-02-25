namespace Box2DSharp
{
    /// The baseSim joint class. Joints are used to constraint two bodies together in
    /// various fashions. Some joints also feature limits and motors.
    public class JointSim
    {
        public int JointId;

        public int BodyIdA;

        public int BodyIdB;

        public JointType Type;

        // Anchors relative to body origin
        public Vec2 LocalOriginAnchorA;

        public Vec2 LocalOriginAnchorB;

        public float InvMassA;

        public float InvMassB;

        public float InvIA;

        public float InvIB;

        public JointUnion Joint;

        public void CopyTo(JointSim jointDst)
        {
            jointDst.JointId = JointId;
            jointDst.BodyIdA = BodyIdA;
            jointDst.BodyIdB = BodyIdB;
            jointDst.Type = Type;
            jointDst.LocalOriginAnchorA = LocalOriginAnchorA;
            jointDst.LocalOriginAnchorB = LocalOriginAnchorB;
            jointDst.InvMassA = InvMassA;
            jointDst.InvMassB = InvMassB;
            jointDst.InvIA = InvIA;
            jointDst.InvIB = InvIB;
            jointDst.Joint = Joint;
        }

        public void Reset()
        {
            JointId = 0;
            BodyIdA = 0;
            BodyIdB = 0;
            Type = default;
            LocalOriginAnchorA = default;
            LocalOriginAnchorB = default;
            InvMassA = default;
            InvMassB = default;
            InvIA = default;
            InvIB = default;
            Joint = default;
        }
    }
}