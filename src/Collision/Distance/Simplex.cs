using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DSharp
{
    /// <summary>
    /// Simplex from the GJK algorithm
    /// </summary>
    public struct Simplex
    {
        public SimplexVertex V1;

        public SimplexVertex V2;

        public SimplexVertex V3;

        /// <summary>
        /// V1,V2,V3 vertices span
        /// </summary>
        public Span<SimplexVertex> Vertices
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => MemoryMarshal.CreateSpan(ref V1, 3);
        }

        /// <summary>
        /// number of valid vertices
        /// </summary>
        public int Count; 
    }
}