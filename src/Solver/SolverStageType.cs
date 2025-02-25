namespace Box2DSharp
{
    /// <summary>
    /// 解算步骤
    /// </summary>
    public enum SolverStageType
    {
        /// <summary>
        /// 准备关节
        /// </summary>
        PrepareJoints,

        /// <summary>
        /// 准备接触点
        /// </summary>
        PrepareContacts,

        /// <summary>
        /// 速度积分
        /// </summary>
        IntegrateVelocities,

        /// <summary>
        /// 热启动
        /// </summary>
        WarmStart,

        /// <summary>
        /// 解算
        /// </summary>
        Solve,

        /// <summary>
        /// 位置积分
        /// </summary>
        IntegratePositions,

        /// <summary>
        /// 松弛
        /// </summary>
        Relax,

        /// <summary>
        /// 弹性计算
        /// </summary>
        Restitution,

        /// <summary>
        /// 储存冲量
        /// </summary>
        StoreImpulses
    }
}