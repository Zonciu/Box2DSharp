using System;

namespace Box2DSharp
{
    /// Contacts and determinism
    /// A deterministic simulation requires contacts to exist in the same order in b2Island no matter the thread count.
    /// The order must reproduce from run to run. This is necessary because the Gauss-Seidel constraint solver is order dependent.
    ///
    /// Creation:
    /// - Contacts are created using results from b2UpdateBroadPhasePairs
    /// - These results are ordered according to the order of the broad-phase move array
    /// - The move array is ordered according to the shape creation order using a bitset.
    /// - The island/shape/body order is determined by creation order
    /// - Logically contacts are only created for awake bodies, so they are immediately added to the awake contact array (serially)
    ///
    /// Island linking:
    /// - The awake contact array is built from the body-contact graph for all awake bodies in awake islands.
    /// - Awake contacts are solved in parallel and they generate contact state changes.
    /// - These state changes may link islands together using union find.
    /// - The state changes are ordered using a bit array that encompasses all contacts
    /// - As long as contacts are created in deterministic order, island link order is deterministic.
    /// - This keeps the order of contacts in islands deterministic
    /// Friction mixing law. The idea is to allow either shape to drive the friction to zero.
    /// For example, anything slides on ice.
    /// 接触和确定性
    /// 确定性模拟要求无论线程数多少，b2Island 中的接触点都必须以相同的顺序存在。
    /// 在每次运行中的顺序要保持一致。这是必要的，因为高斯-赛德尔约束求解器与顺序有关。
    ///
    /// 创建
    /// 使用 b2UpdateBroadPhasePairs 的结果创建接触点
    /// 这个结果根据粗检测移动数组的顺序排列
    /// - 移动数组根据使用比特集储存的形状创建顺序进行排序。
    /// - 岛/形状/刚体的顺序由创建顺序决定
    /// - 从逻辑上讲，只有活跃的刚体才会创建接触点，因此这些接触点会串行地立即添加到活跃接触点数组中
    ///
    /// 岛屿链接：
    /// - 活跃接触点数组是根据活跃岛屿上所有活跃刚体的刚体-接触图建立的。
    /// - 活跃接触点是并行求解的，它们会产生接触状态变化。
    /// - 这些状态变化可以通过联合查找将岛屿连接在一起。
    /// - 这些状态变化通过一个包含所有接触点的比特数组进行排序。
    /// - 只要接触点的创建顺序是确定的，那么岛的链接顺序也是确定的。
    /// - 这就使岛屿中的接触点顺序是确定的。
    /// 摩擦混合规则。其原理是让任何一种形状都能使摩擦力为零。
    /// 例如，任何东西都可以在冰上滑动。
    [Flags]
    public enum ContactFlags
    {
        // Set when the solid shapes are touching.
        ContactTouchingFlag = 0x00000001,

        // Contact has a hit event
        ContactHitEventFlag = 0x00000002,

        // One of the shapes is a sensor
        ContactSensorFlag = 0x0000004,

        // Set when a sensor is touching
        // todo this is not used, perhaps have b2Body_GetSensorContactData()
        ContactSensorTouchingFlag = 0x00000008,

        // This contact wants sensor events
        ContactEnableSensorEvents = 0x00000010,

        // This contact wants contact events
        ContactEnableContactEvents = 0x00000020,
    }
}