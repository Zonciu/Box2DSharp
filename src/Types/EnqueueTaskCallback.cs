namespace Box2DSharp
{
    /// <summary>
    /// These functions can be provided to Box2D to invoke a task system. These are designed to work well with enkiTS.
    /// Returns a pointer to the user's task object. May be nullptr. A nullptr indicates to Box2D that the work was executed
    ///	serially within the callback and there is no need to call b2FinishTaskCallback.
    ///	The itemCount is the number of Box2D work items that are to be partitioned among workers by the user's task system.
    ///	This is essentially a parallel-for. The minRange parameter is a suggestion of the minimum number of items to assign
    ///	per worker to reduce overhead. For example, suppose the task is small and that itemCount is 16. A minRange of 8 suggests
    ///	that your task system should split the work items among just two workers, even if you have more available.
    ///	In general the range [startIndex, endIndex) send to b2TaskCallback should obey:
    ///	endIndex - startIndex >= minRange
    ///	The exception of course is when itemCount &lt; minRange.
    /// <br/>
    /// 这些函数可以提供给Box2D以调用任务系统。
    /// 返回指向用户任务对象的指针。可以是 null。空指针表示工作是在回调中同步串行执行的，没有异步执行，无需调用FinishTaskCallback。
    /// itemCount 是用户任务系统要在 Worker 之间分配的 Box2D 工作项的数量。
    /// 这基本上是一个并行任务。minRange 参数是建议分配给每个工作者的最小项目数，以减少开销。
    /// 每个工作者分配的最小项目数，以减少开销。例如，假设任务较小，itemCount 为 16。如果 minRange 为 8，则建议任务系统应将工作项目分配给两名工人，即使有更多工人可用。
    /// 一般来说，发送给 TaskCallback 的范围 [startIndex, endIndex) 应该符合以下条件：
    /// endIndex - startIndex >= minRange
    /// 当然，itemCount &lt; minRange 时例外。
    /// </summary>
    public delegate object? EnqueueTaskCallback(TaskCallback task, int itemCount, int minRange, object taskContext, object userContext);
}