namespace Box2DSharp
{
    /// <summary>
    /// Task interface
    /// This is prototype for a Box2D task. Your task system is expected to invoke the Box2D task with these arguments.
    /// The task spans a range of the parallel-for: [startIndex, endIndex)
    /// The worker index must correctly identify each worker in the user thread pool, expected in [0, workerCount).
    ///	A worker must only exist on only one thread at a time and is analogous to the thread index.
    /// The task context is the context pointer sent from Box2D when it is enqueued.
    ///	The startIndex and endIndex are expected in the range [0, itemCount) where itemCount is the argument to b2EnqueueTaskCallback
    /// below. Box2D expects startIndex &lt; endIndex and will execute a loop like this:
    ///
    /// 任务回调接口
    /// 这是 Box2D 任务的原型。您的任务系统应使用这些参数调用 Box2D 任务。
    /// 并行任务的范围为：[startIndex, endIndex)
    /// Worker 索引必须正确标识用户线程池中的每个 Worker，预计为 [0, workerCount]。
    /// Worker 每次只能存在于一个线程中，类似于线程索引。
    /// 任务上下文是 Box2D 在排队时发送的上下文指针。
    /// startIndex 和 endIndex 的范围是 [0，itemCount]，其中 itemCount 是下面 EnqueueTaskCallback 的参数。
    /// Box2D 预计 startIndex &lt; endIndex 将执行这样一个循环：
    ///
    /// @code{.c}
    /// for (int i = startIndex; i &lt; endIndex; ++i)
    ///	{
    ///		DoWork();
    ///	}
    ///	@endcode
    /// </summary>
    public delegate void TaskCallback(int startIndex, int endIndex, int workerIndex, object taskContext);
}