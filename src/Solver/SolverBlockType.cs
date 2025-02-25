namespace Box2DSharp
{
    /// <summary>
    /// 解算区块类型
    /// </summary>
    public enum SolverBlockType
    {
        /// <summary>
        /// 刚体区块
        /// </summary>
        BodyBlock,

        /// <summary>
        /// 关节区块
        /// </summary>
        JointBlock,

        /// <summary>
        /// 接触点区块
        /// </summary>
        ContactBlock,

        /// <summary>
        /// 图关节区块
        /// </summary>
        GraphJointBlock,

        /// <summary>
        /// 图接触点区块
        /// </summary>
        GraphContactBlock
    }
}