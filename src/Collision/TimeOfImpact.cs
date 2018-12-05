using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;

namespace Box2DSharp.Collision
{
    /// Input parameters for b2TimeOfImpact
    public struct ToiInput
    {
        public DistanceProxy ProxyA;

        public DistanceProxy ProxyB;

        public Sweep SweepA;

        public Sweep SweepB;

        public float Tmax; // defines sweep interval [0, tMax]
    }

    /// Output parameters for b2TimeOfImpact.
    public struct ToiOutput
    {
        public enum ToiState
        {
            Unknown,

            Failed,

            Overlapped,

            Touching,

            Separated
        }

        public ToiState State;

        public float Time;
    }

    public static class TimeOfImpact
    {
        public static float ToiTime;

        public static float ToiMaxTime;

        public static int ToiCalls;

        public static int ToiIters;

        public static int ToiMaxIters;

        public static int ToiRootIters;

        public static int ToiMaxRootIters;

        /// Compute the upper bound on time before two shapes penetrate. Time is represented as
        /// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
        /// non-tunneling collisions. If you change the time interval, you should call this function
        /// again.
        /// Note: use b2Distance to compute the contact point and normal at the time of impact.
        public static void ComputeTimeOfImpact(out ToiOutput output, in ToiInput input)
        {
            var timer = Stopwatch.StartNew();
            output = new ToiOutput();

            // Todo 只在需要时收集该数据
            ++ToiCalls;

            output.State = ToiOutput.ToiState.Unknown;
            output.Time  = input.Tmax;

            ref readonly var proxyA = ref input.ProxyA;
            ref readonly var proxyB = ref input.ProxyB;

            var sweepA = input.SweepA;
            var sweepB = input.SweepB;

            // Large rotations can make the root finder fail, so we normalize the
            // sweep angles.
            sweepA.Normalize();
            sweepB.Normalize();

            var tMax = input.Tmax;

            var totalRadius = proxyA.Radius + proxyB.Radius;
            var target      = Math.Max(Settings.LinearSlop, totalRadius - 3.0f * Settings.LinearSlop);
            var tolerance   = 0.25f * Settings.LinearSlop;
            Debug.Assert(target > tolerance);

            var       t1            = 0.0f;
            const int maxIterations = 20; // TODO_ERIN b2Settings
            var       iter          = 0;

            // Prepare input for distance query.
            var       cache = new SimplexCache();
            DistanceInput distanceInput;
            distanceInput.ProxyA   = input.ProxyA;
            distanceInput.ProxyB   = input.ProxyB;
            distanceInput.UseRadii = false;

            // The outer loop progressively attempts to compute new separating axes.
            // This loop terminates when an axis is repeated (no progress is made).
            for (;;)
            {
                sweepA.GetTransform(out var xfA, t1);
                sweepB.GetTransform(out var xfB, t1);

                // Get the distance between shapes. We can also use the results
                // to get a separating axis.
                distanceInput.TransformA = xfA;
                distanceInput.TransformB = xfB;

                DistanceAlgorithm.Distance(out var distanceOutput, ref cache, distanceInput);

                // If the shapes are overlapped, we give up on continuous collision.
                if (distanceOutput.Distance <= 0.0f)
                {
                    // Failure!
                    output.State = ToiOutput.ToiState.Overlapped;
                    output.Time  = 0.0f;
                    break;
                }

                if (distanceOutput.Distance < target + tolerance)
                {
                    // Victory!
                    output.State = ToiOutput.ToiState.Touching;
                    output.Time  = t1;
                    break;
                }

                // Initialize the separating axis.
                var fcn = new SeparationFunction();
                fcn.Initialize(
                    ref cache,
                    proxyA,
                    sweepA,
                    proxyB,
                    sweepB,
                    t1);

                // #if 0
                //
                //                 // Dump the curve seen by the root finder
                //                 {
                //                     const int N  = 100;
                //                     float     dx = 1.0f / N;
                //                     float xs[N + 1];
                //                     float fs[N + 1];
                //
                //                     float x = 0.0f;
                //
                //                     for (int i = 0; i <= N; ++i)
                //                     {
                //                         sweepA.GetTransform(&xfA, x);
                //                         sweepB.GetTransform(&xfB, x);
                //                         float f = fcn.Evaluate(xfA, xfB) - target;
                //
                //                         printf("%g %g\n", x, f);
                //
                //                         xs[i] = x;
                //                         fs[i] = f;
                //
                //                         x += dx;
                //                     }
                //                 }
                // #endif

                // Compute the TOI on the separating axis. We do this by successively
                // resolving the deepest point. This loop is bounded by the number of vertices.
                var done         = false;
                var t2           = tMax;
                var pushBackIter = 0;
                for (;;)
                {
                    // Find the deepest point at t2. Store the witness point indices.

                    var s2 = fcn.FindMinSeparation(out var indexA, out var indexB, t2);

                    // Is the final configuration separated?
                    if (s2 > target + tolerance)
                    {
                        // Victory!
                        output.State = ToiOutput.ToiState.Separated;
                        output.Time  = tMax;
                        done         = true;
                        break;
                    }

                    // Has the separation reached tolerance?
                    if (s2 > target - tolerance)
                    {
                        // Advance the sweeps
                        t1 = t2;
                        break;
                    }

                    // Compute the initial separation of the witness points.
                    var s1 = fcn.Evaluate(indexA, indexB, t1);

                    // Check for initial overlap. This might happen if the root finder
                    // runs out of iterations.
                    if (s1 < target - tolerance)
                    {
                        output.State = ToiOutput.ToiState.Failed;
                        output.Time  = t1;
                        done         = true;
                        break;
                    }

                    // Check for touching
                    if (s1 <= target + tolerance)
                    {
                        // Victory! t1 should hold the TOI (could be 0.0).
                        output.State = ToiOutput.ToiState.Touching;
                        output.Time  = t1;
                        done         = true;
                        break;
                    }

                    // Compute 1D root of: f(x) - target = 0
                    var   rootIterCount = 0;
                    float a1            = t1, a2 = t2;
                    for (;;)
                    {
                        // Use a mix of the secant rule and bisection.
                        float t;
                        if ((rootIterCount & 1) != 0)
                        {
                            // Secant rule to improve convergence.
                            t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                        }
                        else
                        {
                            // Bisection to guarantee progress.
                            t = 0.5f * (a1 + a2);
                        }

                        ++rootIterCount;
                        ++ToiRootIters;

                        var s = fcn.Evaluate(indexA, indexB, t);

                        if (Math.Abs(s - target) < tolerance)
                        {
                            // t2 holds a tentative value for t1
                            t2 = t;
                            break;
                        }

                        // Ensure we continue to bracket the root.
                        if (s > target)
                        {
                            a1 = t;
                            s1 = s;
                        }
                        else
                        {
                            a2 = t;
                            s2 = s;
                        }

                        if (rootIterCount == 50)
                        {
                            break;
                        }
                    }

                    ToiMaxRootIters = Math.Max(ToiMaxRootIters, rootIterCount);

                    ++pushBackIter;

                    if (pushBackIter == Settings.MaxPolygonVertices)
                    {
                        break;
                    }
                }

                ++iter;
                ++ToiIters;

                if (done)
                {
                    break;
                }

                if (iter == maxIterations)
                {
                    // Root finder got stuck. Semi-victory.
                    output.State = ToiOutput.ToiState.Failed;
                    output.Time  = t1;
                    break;
                }
            }

            ToiMaxIters = Math.Max(ToiMaxIters, iter);
            timer.Stop();
            float time = timer.ElapsedMilliseconds;
            ToiMaxTime =  Math.Max(ToiMaxTime, time);
            ToiTime    += time;
        }
    }

    //
    public class SeparationFunction
    {
        public enum FunctionType
        {
            e_points,

            e_faceA,

            e_faceB
        }

        public Vector2 m_axis;

        public Vector2 m_localPoint;

        public DistanceProxy m_proxyA;

        public DistanceProxy m_proxyB;

        public Sweep m_sweepA, m_sweepB;

        public FunctionType m_type;

        // TODO_ERIN might not need to return the separation

        public float Initialize(
            ref SimplexCache cache,
            DistanceProxy    proxyA,
            in Sweep         sweepA,
            DistanceProxy    proxyB,
            in Sweep         sweepB,
            float            t1)
        {
            m_proxyA = proxyA;
            m_proxyB = proxyB;
            int count = cache.Count;
            Debug.Assert(0 < count && count < 3);

            m_sweepA = sweepA;
            m_sweepB = sweepB;

            m_sweepA.GetTransform(out var xfA, t1);
            m_sweepB.GetTransform(out var xfB, t1);

            if (count == 1)
            {
                m_type = FunctionType.e_points;
                var localPointA = m_proxyA.GetVertex(cache.IndexA.Value0);
                var localPointB = m_proxyB.GetVertex(cache.IndexB.Value0);
                var pointA      = MathUtils.Mul(xfA, localPointA);
                var pointB      = MathUtils.Mul(xfB, localPointB);
                m_axis = pointB - pointA;
                var s = m_axis.Normalize();
                return s;
            }

            if (cache.IndexA.Value0 == cache.IndexA.Value1)
            {
                // Two points on B and one on A.
                m_type = FunctionType.e_faceB;
                var localPointB1 = proxyB.GetVertex(cache.IndexB.Value0);
                var localPointB2 = proxyB.GetVertex(cache.IndexB.Value1);

                m_axis = MathUtils.Cross(localPointB2 - localPointB1, 1.0f);
                m_axis.Normalize();
                var normal = MathUtils.Mul(xfB.Rotation, m_axis);

                m_localPoint = 0.5f * (localPointB1 + localPointB2);
                var pointB = MathUtils.Mul(xfB, m_localPoint);

                var localPointA = proxyA.GetVertex(cache.IndexA.Value0);
                var pointA      = MathUtils.Mul(xfA, localPointA);

                var s = MathUtils.Dot(pointA - pointB, normal);
                if (s < 0.0f)
                {
                    m_axis = -m_axis;
                    s      = -s;
                }

                return s;
            }
            else
            {
                // Two points on A and one or two points on B.
                m_type = FunctionType.e_faceA;
                var localPointA1 = m_proxyA.GetVertex(cache.IndexA.Value0);
                var localPointA2 = m_proxyA.GetVertex(cache.IndexA.Value1);

                m_axis = MathUtils.Cross(localPointA2 - localPointA1, 1.0f);
                m_axis.Normalize();
                var normal = MathUtils.Mul(xfA.Rotation, m_axis);

                m_localPoint = 0.5f * (localPointA1 + localPointA2);
                var pointA = MathUtils.Mul(xfA, m_localPoint);

                var localPointB = m_proxyB.GetVertex(cache.IndexB.Value0);
                var pointB      = MathUtils.Mul(xfB, localPointB);

                var s = MathUtils.Dot(pointB - pointA, normal);
                if (s < 0.0f)
                {
                    m_axis = -m_axis;
                    s      = -s;
                }

                return s;
            }
        }

        //
        public float FindMinSeparation(out int indexA, out int indexB, float t)
        {
            m_sweepA.GetTransform(out var xfA, t);
            m_sweepB.GetTransform(out var xfB, t);

            switch (m_type)
            {
            case FunctionType.e_points:
            {
                var axisA = MathUtils.MulT(xfA.Rotation, m_axis);
                var axisB = MathUtils.MulT(xfB.Rotation, -m_axis);

                indexA = m_proxyA.GetSupport(axisA);
                indexB = m_proxyB.GetSupport(axisB);

                var localPointA = m_proxyA.GetVertex(indexA);
                var localPointB = m_proxyB.GetVertex(indexB);

                var pointA = MathUtils.Mul(xfA, localPointA);
                var pointB = MathUtils.Mul(xfB, localPointB);

                var separation = MathUtils.Dot(pointB - pointA, m_axis);
                return separation;
            }

            case FunctionType.e_faceA:
            {
                var normal = MathUtils.Mul(xfA.Rotation, m_axis);
                var pointA = MathUtils.Mul(xfA, m_localPoint);

                var axisB = MathUtils.MulT(xfB.Rotation, -normal);

                indexA = -1;
                indexB = m_proxyB.GetSupport(axisB);

                var localPointB = m_proxyB.GetVertex(indexB);
                var pointB      = MathUtils.Mul(xfB, localPointB);

                var separation = MathUtils.Dot(pointB - pointA, normal);
                return separation;
            }

            case FunctionType.e_faceB:
            {
                var normal = MathUtils.Mul(xfB.Rotation, m_axis);
                var pointB = MathUtils.Mul(xfB, m_localPoint);

                var axisA = MathUtils.MulT(xfA.Rotation, -normal);

                indexB = -1;
                indexA = m_proxyA.GetSupport(axisA);

                var localPointA = m_proxyA.GetVertex(indexA);
                var pointA      = MathUtils.Mul(xfA, localPointA);

                var separation = MathUtils.Dot(pointA - pointB, normal);
                return separation;
            }

            default:
                Debug.Assert(false);
                indexA = -1;
                indexB = -1;
                return 0.0f;
            }
        }

        //
        public float Evaluate(int indexA, int indexB, float t)
        {
            m_sweepA.GetTransform(out var xfA, t);
            m_sweepB.GetTransform(out var xfB, t);

            switch (m_type)
            {
            case FunctionType.e_points:
            {
                var localPointA = m_proxyA.GetVertex(indexA);
                var localPointB = m_proxyB.GetVertex(indexB);

                var pointA     = MathUtils.Mul(xfA, localPointA);
                var pointB     = MathUtils.Mul(xfB, localPointB);
                var separation = MathUtils.Dot(pointB - pointA, m_axis);

                return separation;
            }

            case FunctionType.e_faceA:
            {
                var normal = MathUtils.Mul(xfA.Rotation, m_axis);
                var pointA = MathUtils.Mul(xfA, m_localPoint);

                var localPointB = m_proxyB.GetVertex(indexB);
                var pointB      = MathUtils.Mul(xfB, localPointB);

                var separation = MathUtils.Dot(pointB - pointA, normal);
                return separation;
            }

            case FunctionType.e_faceB:
            {
                var normal = MathUtils.Mul(xfB.Rotation, m_axis);
                var pointB = MathUtils.Mul(xfB, m_localPoint);

                var localPointA = m_proxyA.GetVertex(indexA);
                var pointA      = MathUtils.Mul(xfA, localPointA);

                var separation = MathUtils.Dot(pointA - pointB, normal);
                return separation;
            }

            default:
                Debug.Assert(false);
                return 0.0f;
            }
        }
    }
}