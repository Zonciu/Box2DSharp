namespace Box2DSharp
{
    /// <summary>
    /// Each stage must be completed before going to the next stage.
    /// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
    /// </summary>
    public class SolverStage
    {
        public SolverStageType Type;

        public SolverBlock[] Blocks = null!;

        public int BlockCount;

        public int ColorIndex;

        /// <summary>
        /// Atomic, todo consider false sharing of this atomic
        /// </summary>
        public int CompletionCount;
    }
}