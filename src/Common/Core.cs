using System.Diagnostics;
using System.Numerics;

namespace Box2DSharp
{
    public static class Core
    {
        public const int NullIndex = -1;

#if DEBUG
        public const bool B2Debug = true;
#else
		public const bool B2Debug = false;
#endif

#if BOX2D_VALIDATE
        public const bool B2Validate = B2Debug;
#else
        public const bool B2Validate = false;
#endif
        public static readonly int BodyTypeCount = (int)BodyType.BodyTypeCount;

        public const int InitialCapacity = 16;

        public const bool ForceOverflow = true;

        public const int OverflowIndex = GraphColorCount - 1;

        public static readonly int SimdWidth = Vector<int>.Count;

        public static readonly int SimdShift = SimdWidth == 8 ? 3 : 2;

        /// <summary>
        /// The maximum number of vertices on a convex polygon. Changing this affects performance even if you
        ///	don't use more vertices.
        /// </summary>
        public const int MaxPolygonVertices = 8;

        /// <summary>
        /// The default category bit for a tree proxy. Used for collision filtering.
        /// </summary>
        public const int DefaultCategoryBits = 1;

        /// <summary>
        /// Convenience mask bits to use when you don't need collision filtering and just want
        ///	all results.
        /// </summary>
        public const ulong DefaultMaskBits = ulong.MaxValue;

        /// <summary>
        /// 真实世界每米对应的模拟长度单位
        /// </summary>
        public static float LengthUnitsPerMeter = 1.0f;

        /// <summary>
        /// Used to detect bad values. Positions greater than about 16km will have precision
        /// problems, so 100km as a limit should be fine in all cases.
        /// </summary>
        public static float Huge = 100000.0f * LengthUnitsPerMeter;

        /// <summary>
        /// Maximum parallel workers. Used to size some static arrays.
        /// </summary>
        public const int MaxWorkers = 64;

        /// <summary>
        /// Maximum number of colors in the constraint graph. Constraints that cannot
        /// find a color are added to the overflow set which are solved single-threaded.
        /// </summary>
        public const int GraphColorCount = 12;

        /// <summary>
        /// A small length used as a collision and constraint tolerance. Usually it is
        /// chosen to be numerically significant, but visually insignificant. In meters.
        /// @warning modifying this can have a significant impact on stability
        /// </summary>
        public static float LinearSlop = 0.005f * LengthUnitsPerMeter;

        /// <summary>
        /// Maximum number of simultaneous worlds that can be allocated
        /// </summary>
        public const int MaxWorlds = 128;

        /// <summary>
        /// The maximum rotation of a body per time step. This limit is very large and is used
        /// to prevent numerical problems. You shouldn't need to adjust this.
        /// @warning increasing this to 0.5f * b2_pi or greater will break continuous collision.
        /// </summary>
        public const float MaxRotation = 0.25f * B2Math.Pi;

        /// <summary>
        /// @warning modifying this can have a significant impact on performance and stability
        /// </summary>
        public static float SpeculativeDistance = 4.0f * LinearSlop;

        /// <summary>
        /// This is used to fatten AABBs in the dynamic tree. This allows proxies
        /// to move by a small amount without triggering a tree adjustment.
        /// This is in meters.
        /// @warning modifying this can have a significant impact on performance
        /// </summary>
        public static float b2_aabbMargin = 0.1f * LengthUnitsPerMeter;

        /// <summary>
        /// The time that a body must be still before it will go to sleep. In seconds.
        /// </summary>
        public const float TimeToSleep = 0.5f;

        /// <summary>
        /// Use to validate definitions. Do not take my cookie.
        /// </summary>
        public const int SecretCookie = 1152023;

        #region CheckDef

        public static void CheckDef(in this WorldDef def)
        {
            Debug.Assert(def.InternalValue == SecretCookie);
        }

        public static void CheckDef(in this BodyDef def)
        {
            Debug.Assert(def.InternalValue == SecretCookie);
        }

        public static void CheckDef(in this ShapeDef def)
        {
            Debug.Assert(def.InternalValue == SecretCookie);
        }

        public static void CheckDef(in this ChainDef def)
        {
            Debug.Assert(def.InternalValue == SecretCookie);
        }

        public static void CheckDef(in this DistanceJointDef def)
        {
            Debug.Assert(def.InternalValue == SecretCookie);
        }

        public static void CheckDef(in this MotorJointDef def)
        {
            Debug.Assert(def.InternalValue == SecretCookie);
        }

        public static void CheckDef(in this MouseJointDef def)
        {
            Debug.Assert(def.InternalValue == SecretCookie);
        }

        public static void CheckDef(in this PrismaticJointDef def)
        {
            Debug.Assert(def.InternalValue == SecretCookie);
        }

        public static void CheckDef(in this RevoluteJointDef def)
        {
            Debug.Assert(def.InternalValue == SecretCookie);
        }

        public static void CheckDef(in this WeldJointDef def)
        {
            Debug.Assert(def.InternalValue == SecretCookie);
        }

        public static void CheckDef(in this WheelJointDef def)
        {
            Debug.Assert(def.InternalValue == SecretCookie);
        }

        #endregion

        public static void SetLengthUnitsPerMeter(float lengthUnits)
        {
            Debug.Assert(B2Math.IsValid(lengthUnits) && lengthUnits > 0.0f);
            LengthUnitsPerMeter = lengthUnits;
        }

        public static float GetLengthUnitsPerMeter()
        {
            return LengthUnitsPerMeter;
        }

        /// <summary>
        /// 获取Box2D版本号
        /// </summary>
        /// <returns></returns>
        public static Version GetVersion()
        {
            return new Version(3, 1, 0);
        }

        /// <summary>
        /// 组合Shape的Id到一个ulong值中
        /// </summary>
        /// <param name="k1"></param>
        /// <param name="k2"></param>
        /// <returns></returns>
        public static ulong ShapePairKey(int k1, int k2)
        {
            return k1 < k2 ? (ulong)k1 << 32 | (uint)k2 : (ulong)k2 << 32 | (uint)k1;
        }
    }
}