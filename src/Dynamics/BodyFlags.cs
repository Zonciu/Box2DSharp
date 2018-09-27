using System;

namespace Box2DSharp.Dynamics
{
    [Flags]
    public enum BodyFlags
    {
        /// <summary>
        /// 孤岛
        /// </summary>
        Island = 0x0001,

        /// <summary>
        /// 醒着的
        /// </summary>
        IsAwake = 0x0002,

        /// <summary>
        /// 自动休眠
        /// </summary>
        AutoSleep = 0x0004,

        /// <summary>
        /// 子弹
        /// </summary>
        IsBullet = 0x0008,

        /// <summary>
        /// </summary>
        FixedRotation = 0x0010,

        /// <summary>
        /// 活跃
        /// </summary>
        IsActive = 0x0020,

        /// <summary>
        /// 碰撞时间
        /// </summary>
        Toi = 0x0040
    }
}