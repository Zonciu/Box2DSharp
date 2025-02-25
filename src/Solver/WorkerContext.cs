namespace Box2DSharp
{
    /// <summary>
    /// 工人上下文
    /// </summary>
    public struct WorkerContext
    {
        /// <summary>
        /// 所属步进上下文
        /// </summary>
        public StepContext Context;

        /// <summary>
        /// 工人索引
        /// </summary>
        public int WorkerIndex;
    }
}