using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Collision;

[Sample("Collision", "DynamicTree")]
public class DynamicTreeSample : SampleBase
{
    protected DynamicTree Tree;

    protected int RowCount;

    protected int ColumnCount;

    protected Proxy[] Proxies;

    protected int[] MoveBuffer;

    protected int MoveCount;

    protected int ProxyCapacity;

    protected int ProxyCount;

    protected int TimeStamp;

    protected int UpdateType;

    protected float Fill;

    protected float MoveFraction;

    protected float MoveDelta;

    protected float Ratio;

    protected float Grid;

    protected Vec2 StartPoint;

    protected Vec2 EndPoint;

    protected bool RayDrag;

    protected bool QueryDrag;

    protected bool Validate;

    protected static class UpdateTypes
    {
        public const int Incremental = 0;

        public const int FullRebuild = 1;

        public const int PartialRebuild = 2;
    }

    protected struct Proxy
    {
        public AABB Box;

        public AABB FatBox;

        public Vec2 Position;

        public Vec2 Width;

        public int ProxyId;

        public int RayStamp;

        public int QueryStamp;

        public bool Moved;
    }

    // Tests the Box2D bounding volume hierarchy (BVH). The dynamic tree
    // can be used independently as a spatial data structure.
    public DynamicTreeSample(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (500.0f, 500.0f);
            Global.Camera.Zoom = 25.0f * 21.0f;
        }

        Fill = 0.25f;
        MoveFraction = 0.05f;
        MoveDelta = 0.1f;
        Proxies = null;
        ProxyCount = 0;
        ProxyCapacity = 0;
        Ratio = 5.0f;
        Grid = 1.0f;

        MoveBuffer = null;
        MoveCount = 0;

        RowCount = Core.B2Debug ? 100 : 1000;
        ColumnCount = Core.B2Debug ? 100 : 1000;
        BuildTree();
        TimeStamp = 0;
        UpdateType = UpdateTypes.Incremental;

        StartPoint = (0.0f, 0.0f);
        EndPoint = (0.0f, 0.0f);
        QueryDrag = false;
        RayDrag = false;
        Validate = true;
    }

    public override void Dispose()
    {
        base.Dispose();

        Proxies = null;
        MoveBuffer = null;
        Tree?.Destroy();
    }

    protected void BuildTree()
    {
        Tree?.Destroy();
        ProxyCapacity = RowCount * ColumnCount;
        if (Proxies == null!)
        {
            Proxies = new Proxy[ProxyCapacity];
        }
        else
        {
            Array.Clear(Proxies, 0, Proxies.Length);
        }

        ProxyCount = 0;

        if (MoveBuffer == null!)
        {
            MoveBuffer = new int[ProxyCapacity];
        }
        else
        {
            Array.Clear(MoveBuffer, 0, MoveBuffer.Length);
        }

        MoveCount = 0;

        float y = -4.0f;

        bool isStatic = false;
        Tree = new DynamicTree();

        Vec2 aabbMargin = (0.1f, 0.1f);

        for (int i = 0; i < RowCount; ++i)
        {
            float x = -40.0f;

            for (int j = 0; j < ColumnCount; ++j)
            {
                float fillTest = RandomFloat(0.0f, 1.0f);
                if (fillTest <= Fill)
                {
                    Debug.Assert(ProxyCount <= ProxyCapacity);
                    ref Proxy p = ref Proxies[ProxyCount];
                    p.Position = (x, y);

                    float ratio = RandomFloat(1.0f, Ratio);
                    float width = RandomFloat(0.1f, 0.5f);
                    if (RandomFloat() > 0.0f)
                    {
                        p.Width.X = ratio * width;
                        p.Width.Y = width;
                    }
                    else
                    {
                        p.Width.X = width;
                        p.Width.Y = ratio * width;
                    }

                    p.Box.LowerBound = (x, y);
                    p.Box.UpperBound = (x + p.Width.X, y + p.Width.Y);
                    p.FatBox.LowerBound = p.Box.LowerBound - aabbMargin;
                    p.FatBox.UpperBound = p.Box.UpperBound + aabbMargin;

                    p.ProxyId = Tree.CreateProxy(p.FatBox, Core.DefaultCategoryBits, ProxyCount);
                    p.RayStamp = -1;
                    p.QueryStamp = -1;
                    p.Moved = false;
                    ++ProxyCount;
                }

                x += Grid;
            }

            y += Grid;
        }
    }

    public override void MouseDown(Vec2 p, MouseInputEventArgs e)
    {
        base.MouseDown(p, e);
        if (e.Button == MouseButton.Left)
        {
            if (!e.Modifiers.IsSet(KeyModifiers.Shift) && QueryDrag == false)
            {
                RayDrag = true;
                StartPoint = p;
                EndPoint = p;
            }
            else if (e.Modifiers.IsSet(KeyModifiers.Shift) && RayDrag == false)
            {
                QueryDrag = true;
                StartPoint = p;
                EndPoint = p;
            }
        }
    }

    public override void MouseUp(Vec2 p, MouseInputEventArgs e)
    {
        base.MouseUp(p, e);
        if (e.Button == MouseButton.Left)
        {
            QueryDrag = false;
            RayDrag = false;
        }
    }

    public override void MouseMove(Vec2 p, MouseMoveEventArgs e)
    {
        base.MouseMove(p, e);
        EndPoint = p;
    }

    public override void PostStep()
    {
        if (QueryDrag)
        {
            AABB box = (B2Math.Min(StartPoint, EndPoint), B2Math.Max(StartPoint, EndPoint));
            var dynamicTreeSample = this;
            Tree.Query(box, Core.DefaultMaskBits, QueryCallback, dynamicTreeSample);

            Draw.DrawAABB(box, B2HexColor.White);
        }

        if (RayDrag)
        {
            RayCastInput input = new(StartPoint, EndPoint - StartPoint, 1.0f);
            var dynamicTreeSample = this;
            Tree.RayCast(ref input, Core.DefaultMaskBits, RayCallback, dynamicTreeSample);

            Draw.DrawSegment(StartPoint, EndPoint, B2HexColor.White);
            Draw.DrawPoint(StartPoint, 5.0f, B2HexColor.Green);
            Draw.DrawPoint(EndPoint, 5.0f, B2HexColor.Red);
        }

        B2HexColor c = B2HexColor.Blue;
        B2HexColor qc = B2HexColor.Green;

        Vec2 aabbMargin = new(0.1f, 0.1f);

        for (int i = 0; i < ProxyCount; ++i)
        {
            ref Proxy p = ref Proxies[i];

            if (p.QueryStamp == TimeStamp || p.RayStamp == TimeStamp)
            {
                Draw.DrawAABB(p.Box, qc);
            }
            else
            {
                Draw.DrawAABB(p.Box, c);
            }

            float moveTest = RandomFloat(0.0f, 1.0f);
            if (MoveFraction > moveTest)
            {
                float dx = MoveDelta * RandomFloat();
                float dy = MoveDelta * RandomFloat();

                p.Position.X += dx;
                p.Position.Y += dy;

                p.Box.LowerBound.X = p.Position.X + dx;
                p.Box.LowerBound.Y = p.Position.Y + dy;
                p.Box.UpperBound.X = p.Position.X + dx + p.Width.X;
                p.Box.UpperBound.Y = p.Position.Y + dy + p.Width.Y;

                if (AABB.Contains(p.FatBox, p.Box) == false)
                {
                    p.FatBox.LowerBound = p.Box.LowerBound - aabbMargin;
                    p.FatBox.UpperBound = p.Box.UpperBound + aabbMargin;
                    p.Moved = true;
                }
                else
                {
                    p.Moved = false;
                }
            }
            else
            {
                p.Moved = false;
            }
        }

        switch (UpdateType)
        {
        case UpdateTypes.Incremental:
        {
            var timer = Stopwatch.StartNew();
            for (int i = 0; i < ProxyCount; ++i)
            {
                ref readonly Proxy p = ref Proxies[i];
                if (p.Moved)
                {
                    Tree.MoveProxy(p.ProxyId, p.FatBox);
                }
            }

            float ms = timer.ElapsedMilliseconds;
            DrawString($"incremental : {ms:F3} ms");
        }
            break;

        case UpdateTypes.FullRebuild:
        {
            for (int i = 0; i < ProxyCount; ++i)
            {
                ref readonly Proxy p = ref Proxies[i];
                if (p.Moved)
                {
                    Tree.EnlargeProxy(p.ProxyId, p.FatBox);
                }
            }

            var timer = Stopwatch.StartNew();
            int boxCount = Tree.Rebuild(true);
            float ms = timer.ElapsedMilliseconds;
            DrawString($"full build {boxCount} : {ms:F3} ms");
        }
            break;

        case UpdateTypes.PartialRebuild:
        {
            for (int i = 0; i < ProxyCount; ++i)
            {
                ref readonly Proxy p = ref Proxies[i];
                if (p.Moved)
                {
                    Tree.EnlargeProxy(p.ProxyId, p.FatBox);
                }
            }

            var timer = Stopwatch.StartNew();
            int boxCount = Tree.Rebuild(false);
            float ms = timer.ElapsedMilliseconds;
            DrawString($"partial rebuild {boxCount} : {ms:F3} ms");
        }
            break;

        default:
            break;
        }

        int height = Tree.GetHeight();
        float areaRatio = Tree.GetAreaRatio();

        int hmin = (int)(Math.Ceiling(Math.Log((float)ProxyCount) / Math.Log(2.0f) - 1.0f));
        DrawString($"proxies = {ProxyCount}, height = {height}, hmin = {hmin}, area ratio = {areaRatio:F1}");

        Tree.Validate();

        TimeStamp += 1;
    }

    private static bool QueryCallback(int proxyId, int userData, object context)
    {
        DynamicTreeSample sample = (DynamicTreeSample)context;
        ref Proxy proxy = ref sample.Proxies[userData];
        Debug.Assert(proxy.ProxyId == proxyId);
        proxy.QueryStamp = sample.TimeStamp;
        return true;
    }

    private static float RayCallback(ref RayCastInput input, int proxyId, int userData, object context)
    {
        DynamicTreeSample sample = (DynamicTreeSample)context;
        ref Proxy proxy = ref sample.Proxies[userData];
        Debug.Assert(proxy.ProxyId == proxyId);
        proxy.RayStamp = sample.TimeStamp;
        return input.MaxFraction;
    }
}