using System;

namespace Box2DSharp
{
    /// <summary>
    /// Context for a time step. Recreated each time step.
    /// </summary>
    public class StepContext
    {
        /// <summary>
        /// time step
        /// 单步经历的时间
        /// </summary>
        public float Dt;

        /// <summary>
        /// inverse time step (0 if dt == 0).
        /// t的倒数，如果Dt是0，InvDt也记为0
        /// </summary>
        public float InvDt;

        /// <summary>
        /// sub-step
        /// 子步时间分片数，等于单步时间/子步数 (Dt / SubStepCount)
        /// </summary>
        public float H;

        /// <summary>
        /// 子步时间分片数的倒数
        /// </summary>
        public float InvH;

        /// <summary>
        /// 子步数量
        /// </summary>
        public int SubStepCount;

        public Softness JointSoftness;

        public Softness ContactSoftness;

        public Softness StaticSoftness;

        /// <summary>
        /// 弹性阈值
        /// </summary>
        public float RestitutionThreshold;

        /// <summary>
        /// 最大速度
        /// </summary>
        public float MaxLinearVelocity;

        /// <summary>
        /// Step上下文对应的世界
        /// </summary>
        public World World = null!;

        /// <summary>
        /// 约束图
        /// </summary>
        public ConstraintGraph Graph = null!;

        /// <summary>
        /// shortcut to body states from awake set
        /// 活跃集合的刚体状态
        /// </summary>
        public B2Array<BodyState> States = null!;

        /// <summary>
        /// shortcut to body sims from awake set
        /// 活跃集合的刚体模拟数据
        /// </summary>
        public B2Array<BodySim> Sims = null!;

        /// <summary>
        /// array of all shape ids for shapes that have enlarged AABBs
        /// 具有扩大化AABB的形状的Id集合
        /// </summary>
        public int[] EnlargedShapes = null!;

        public int EnlargedShapeCount;

        /// <summary>
        /// Array of fast bodies that need continuous collision handling
        /// 需要连续碰撞处理的高速刚体集合
        /// </summary>
        public int[] FastBodies = null!;

        /// <summary>
        /// 高速物体数量，读写时需要原子操作
        /// </summary>
        public int FastBodyCount;

        /// <summary>
        /// Array of bullet bodies that need continuous collision handling
        /// 需要连续碰撞处理的子弹刚体
        /// </summary>
        public int[] BulletBodies = null!;

        /// <summary>
        /// 子弹刚体数量，读写需要原子操作
        /// </summary>
        public int BulletBodyCount;

        /// <summary>
        /// joint pointers for simplified parallel-for access.
        /// 用于简化并行访问操作的关节引用
        /// </summary>
        public JointSim[] Joints = null!;

        /// <summary>
        /// contact pointers for simplified parallel-for access.
        /// <br/> - parallel-for collide with no gaps
        /// <br/> - parallel-for prepare and store contacts with null gaps for SIMD remainders
        /// despite being an array of pointers, these are contiguous sub-arrays corresponding
        /// to constraint graph colors
        /// <br/> 用于简化并行访问操作的接触点引用
        /// <br/> - 并行计算无间隙碰撞
        /// <br/> - 储存SIMD之外的无间隙接触点，尽管它是一个指针数组，但这些是对应于约束图颜色的连续子数组
        /// </summary>
        public Memory<ContactSim> Contacts;

        /// <summary>
        /// SIMD接触约束
        /// </summary>
        public ContactConstraintSIMD[] SimdContactConstraints = null!;

        /// <summary>
        /// 活跃颜色数
        /// </summary>
        public int ActiveColorCount;

        /// <summary>
        /// 并行工人数
        /// </summary>
        public int WorkerCount;

        /// <summary>
        /// 解算步骤
        /// </summary>
        public SolverStage[] Stages = null!;

        /// <summary>
        /// 解算步骤数量
        /// </summary>
        public int StageCount;

        /// <summary>
        /// 允许热启动
        /// </summary>
        public bool EnableWarmStarting;

        // todo padding to prevent false sharing
        //public char dummy1[64];

        /// <summary>
        /// _Atomic, sync index (16-bits) | stage type (16-bits)
        /// 同步索引 (16-bits) | 步骤类型 (16-bits)，读写需要原子操作
        /// </summary>
        public uint AtomicSyncBits;

        // public char dummy2[64];
    }
}