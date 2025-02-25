namespace Box2DSharp
{
    /// <summary>
    /// Body simulation data used for integration of position and velocity<br/>
    /// Transform data used for collision and solver preparation.<br/>
    /// 刚体仿真数据，用于位置和速度的积分计算，Transform用于碰撞和解算
    /// </summary>
    public class BodySim
    {
        /// <summary>
        /// todo better to have transform in sim or in base body? Try both!
        /// transform for body origin
        /// 刚体原点的位置、角度
        /// </summary>
        public Transform Transform;

        /// <summary>
        /// center of mass position in world space
        /// 世界坐标系中的质心坐标
        /// </summary>
        public Vec2 Center;

        /// <summary>
        /// 上次的角度，用于TOI检查
        /// </summary>
        public Rot Rotation0;

        /// <summary>
        /// 上一次的质心坐标(世界坐标系)，用于TOI检查
        /// </summary>
        public Vec2 Center0;

        /// <summary>
        /// location of center of mass relative to the body origin
        /// 相对于刚体原点的质心本地坐标(以刚体原点为坐标原点)
        /// </summary>
        public Vec2 LocalCenter;

        /// <summary>
        /// 受力
        /// </summary>
        public Vec2 Force;

        /// <summary>
        /// 旋转力矩
        /// </summary>
        public float Torque;

        /// <summary>
        /// 质量
        /// </summary>
        public float Mass;

        /// <summary>
        /// 质量倒数
        /// </summary>
        public float InvMass;

        /// <summary>
        /// 质心转动惯量
        /// </summary>
        public float Inertia;

        /// <summary>
        /// 质心转动惯量的倒数
        /// </summary>
        public float InvInertia;

        public float MinExtent;

        public float MaxExtent;

        /// <summary>
        /// 线性阻尼
        /// </summary>
        public float LinearDamping;

        /// <summary>
        /// 角度阻尼
        /// </summary>
        public float AngularDamping;

        /// <summary>
        /// 重力缩放比例
        /// </summary>
        public float GravityScale;

        /// <summary>
        /// 刚体Id
        /// body data can be moved around, the id is stable (used in b2BodyId) 
        /// </summary>
        public int BodyId;

        /// <summary>
        /// 高速刚体标识
        /// todo eliminate
        /// </summary>
        public bool IsFast;

        /// <summary>
        /// 子弹标识
        /// </summary>
        public bool IsBullet;

        /// <summary>
        /// 线速度或角速度超过限值
        /// </summary>
        public bool IsSpeedCapped;

        /// <summary>
        /// 允许快速旋转
        /// </summary>
        public bool AllowFastRotation;

        /// <summary>
        /// AABB扩大
        /// </summary>
        public bool EnlargeAABB;

        public BodySim()
        { }

        public static Sweep MakeSweep(BodySim bodySim)
        {
            Sweep s = new()
            {
                C1 = bodySim.Center0,
                C2 = bodySim.Center,
                Q1 = bodySim.Rotation0,
                Q2 = bodySim.Transform.Q,
                LocalCenter = bodySim.LocalCenter
            };
            return s;
        }

        public void CopyTo(BodySim simDst)
        {
            simDst.Transform = Transform;
            simDst.Center = Center;
            simDst.Rotation0 = Rotation0;
            simDst.Center0 = Center0;
            simDst.LocalCenter = LocalCenter;
            simDst.Force = Force;
            simDst.Torque = Torque;
            simDst.Mass = Mass;
            simDst.InvMass = InvMass;
            simDst.Inertia = Inertia;
            simDst.InvInertia = InvInertia;
            simDst.MinExtent = MinExtent;
            simDst.MaxExtent = MaxExtent;
            simDst.LinearDamping = LinearDamping;
            simDst.AngularDamping = AngularDamping;
            simDst.GravityScale = GravityScale;
            simDst.BodyId = BodyId;
            simDst.IsFast = IsFast;
            simDst.IsBullet = IsBullet;
            simDst.IsSpeedCapped = IsSpeedCapped;
            simDst.AllowFastRotation = AllowFastRotation;
            simDst.EnlargeAABB = EnlargeAABB;
        }

        public void Reset()
        {
            Transform = default;
            Center = default;
            Rotation0 = default;
            Center0 = default;
            LocalCenter = default;
            Force = default;
            Torque = default;
            Mass = default;
            InvMass = default;
            Inertia = default;
            InvInertia = default;
            MinExtent = default;
            MaxExtent = default;
            LinearDamping = default;
            AngularDamping = default;
            GravityScale = default;
            BodyId = default;
            IsFast = default;
            IsBullet = default;
            IsSpeedCapped = default;
            AllowFastRotation = default;
            EnlargeAABB = default;
        }
    }
}