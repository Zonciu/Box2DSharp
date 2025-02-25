using System;
using Box2DSharp;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using Testbed.Abstractions;

namespace Testbed.Render;

// Rounded and non-rounded convex polygons using an SDF-based shader.
public class GLSolidPolygons : IDisposable
{
    private const int MaxCount = 512;

    private readonly B2Array<PolygonData> _polygons = new(MaxCount);

    private int _vaoId;

    private readonly int[] _vboIds = new int[2];

    private int _programId;

    private readonly int _projectionUniform;

    private readonly int _pixelScaleUniform;

    private struct PolygonData(Transform transform, FixedArray8<Vec2> p, int count, float radius, Color4 color)
    {
        public Transform Transform = transform;

        public FixedArray8<Vec2> P = p;

        public int Count = count;

        public float Radius = radius;

        public Color4 Color = color;
    }

    public GLSolidPolygons()
    {
        _programId = RenderHelper.CreateProgramFromFiles("Render/data/solid_polygon.vs", "Render/data/solid_polygon.fs");

        _projectionUniform = GL.GetUniformLocation(_programId, "projectionMatrix");
        _pixelScaleUniform = GL.GetUniformLocation(_programId, "pixelScale");
        int vertexAttribute = 0;
        int instanceTransform = 1;
        int instancePoint12 = 2;
        int instancePoint34 = 3;
        int instancePoint56 = 4;
        int instancePoint78 = 5;
        int instancePointCount = 6;
        int instanceRadius = 7;
        int instanceColor = 8;

        // Generate
        GL.GenVertexArrays(1, out _vaoId);
        GL.GenBuffers(2, _vboIds);

        GL.BindVertexArray(_vaoId);
        GL.EnableVertexAttribArray(vertexAttribute);
        GL.EnableVertexAttribArray(instanceTransform);
        GL.EnableVertexAttribArray(instancePoint12);
        GL.EnableVertexAttribArray(instancePoint34);
        GL.EnableVertexAttribArray(instancePoint56);
        GL.EnableVertexAttribArray(instancePoint78);
        GL.EnableVertexAttribArray(instancePointCount);
        GL.EnableVertexAttribArray(instanceRadius);
        GL.EnableVertexAttribArray(instanceColor);

        // Vertex buffer for sinGL.e quad
        float a = 1.1f;
        Vec2[] vertices = [(-a, -a), (a, -a), (-a, a), (a, -a), (a, a), (-a, a)];
        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[0]);
        GL.BufferData(BufferTarget.ArrayBuffer, SizeCache<Vec2>.Size * vertices.Length, vertices, BufferUsageHint.StaticDraw);
        GL.VertexAttribPointer(vertexAttribute, 2, VertexAttribPointerType.Float, false, 0, 0);

        // Polygon buffer
        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[1]);
        GL.BufferData(BufferTarget.ArrayBuffer, MaxCount * SizeCache<PolygonData>.Size, 0, BufferUsageHint.DynamicDraw);

        GL.VertexAttribPointer(instanceTransform, 4, VertexAttribPointerType.Float, false, SizeCache<PolygonData>.Size, 0);
        GL.VertexAttribPointer(instancePoint12, 4, VertexAttribPointerType.Float, false, SizeCache<PolygonData>.Size, 16);
        GL.VertexAttribPointer(instancePoint34, 4, VertexAttribPointerType.Float, false, SizeCache<PolygonData>.Size, 32);
        GL.VertexAttribPointer(instancePoint56, 4, VertexAttribPointerType.Float, false, SizeCache<PolygonData>.Size, 48);
        GL.VertexAttribPointer(instancePoint78, 4, VertexAttribPointerType.Float, false, SizeCache<PolygonData>.Size, 64);
        GL.VertexAttribIPointer(instancePointCount, 1, VertexAttribIntegerType.Int, SizeCache<PolygonData>.Size, 80);
        GL.VertexAttribPointer(instanceRadius, 1, VertexAttribPointerType.Float, false, SizeCache<PolygonData>.Size, 84);

        // color will get automatically expanded to floats in the shader
        GL.VertexAttribPointer(instanceColor, 4, VertexAttribPointerType.Float, true, SizeCache<PolygonData>.Size, 88);

        // These divisors tell GL.sl how to distribute per instance data
        GL.VertexAttribDivisor(instanceTransform, 1);
        GL.VertexAttribDivisor(instancePoint12, 1);
        GL.VertexAttribDivisor(instancePoint34, 1);
        GL.VertexAttribDivisor(instancePoint56, 1);
        GL.VertexAttribDivisor(instancePoint78, 1);
        GL.VertexAttribDivisor(instancePointCount, 1);
        GL.VertexAttribDivisor(instanceRadius, 1);
        GL.VertexAttribDivisor(instanceColor, 1);

        RenderHelper.CheckGLError();

        // Cleanup
        GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
        GL.BindVertexArray(0);
    }

    public void Dispose()
    {
        if (_vaoId != 0)
        {
            GL.DeleteVertexArrays(1, ref _vaoId);
            GL.DeleteBuffers(2, _vboIds);
            _vaoId = 0;
        }

        if (_programId != 0)
        {
            GL.DeleteProgram(_programId);
            _programId = 0;
        }
    }

    public void AddPolygon(in Transform transform, in ReadOnlySpan<Vec2> points, int count, float radius, Color4 color)
    {
        PolygonData data = new()
        {
            Transform = transform,
            Radius = radius,
            Color = color
        };

        int n = count < 8 ? count : 8;
        var ps = data.P.Span;
        for (int i = 0; i < n; ++i)
        {
            ps[i] = points[i];
        }

        data.Count = n;

        _polygons.Push(data);
    }

    public void Flush()
    {
        int count = _polygons.Count;
        if (count == 0)
        {
            return;
        }

        GL.UseProgram(_programId);

        Span<float> proj = stackalloc float[16];
        Global.Camera.BuildProjectionMatrix(proj, 0.2f);

        GL.UniformMatrix4(_projectionUniform, 1, false, ref proj[0]);
        GL.Uniform1(_pixelScaleUniform, Global.Camera.Height / Global.Camera.Zoom);

        GL.BindVertexArray(_vaoId);
        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[1]);

        GL.Enable(EnableCap.Blend);
        GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

        int @base = 0;
        while (count > 0)
        {
            int batchCount = Math.Min(count, MaxCount);

            GL.BufferSubData(BufferTarget.ArrayBuffer, 0, batchCount * SizeCache<PolygonData>.Size, ref _polygons[@base]);
            GL.DrawArraysInstanced(PrimitiveType.Triangles, 0, 6, batchCount);
            RenderHelper.CheckGLError();

            count -= MaxCount;
            @base += MaxCount;
        }

        GL.Disable(EnableCap.Blend);

        GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
        GL.BindVertexArray(0);
        GL.UseProgram(0);

        _polygons.Clear();
    }
}