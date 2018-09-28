using System.Numerics;
using Box2DSharp.Common;

namespace Box2DSharp.Collision.Collider
{
    /// A manifold for two touching convex shapes.
    /// Box2D supports multiple types of contact:
    /// - clip point versus plane with radius
    /// - point versus point with radius (circles)
    /// The local point usage depends on the manifold type:
    /// -e_circles: the local center of circleA
    /// -e_faceA: the center of faceA
    /// -e_faceB: the center of faceB
    /// Similarly the local normal usage:
    /// -e_circles: not used
    /// -e_faceA: the normal on polygonA
    /// -e_faceB: the normal on polygonB
    /// We store contacts in this way so that position correction can
    /// account for movement, which is critical for continuous physics.
    /// All contact scenarios must be expressed in one of these types.
    /// This structure is stored across time steps, so we keep it small.
    public class Manifold
    {
        /// the points of contact
        public readonly ManifoldPoint[] Points = new ManifoldPoint[Settings.MaxManifoldPoints];

        /// not use for Type::e_points
        public Vector2 LocalNormal = Vector2.Zero;

        /// usage depends on manifold type
        public Vector2 LocalPoint = Vector2.Zero;

        public ManifoldType Type = 0;

        /// the number of manifold points
        public int PointCount = 0;
    }
}