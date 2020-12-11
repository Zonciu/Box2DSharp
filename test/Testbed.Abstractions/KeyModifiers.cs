using System;

namespace Testbed.Abstractions
{
    [Flags]
    public enum KeyModifiers
    {
        /// <summary>if one or more Shift keys were held down.</summary>
        Shift = 1 << 0,

        /// <summary>If one or more Control keys were held down.</summary>
        Control = 1 << 1,

        /// <summary>If one or more Alt keys were held down.</summary>
        Alt = 1 << 2,

        /// <summary>If one or more Super keys were held down.</summary>
        Super = 1 << 3,

        /// <summary>If caps lock is enabled.</summary>
        CapsLock = 1 << 4,

        /// <summary>If num lock is enabled.</summary>
        NumLock = 1 << 5
    }
}