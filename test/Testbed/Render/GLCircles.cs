using System;
using Box2DSharp;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using Testbed.Abstractions;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.Render;

public class GLCircles : IDisposable
{
    public const int MaxCount = 2048;

    private readonly B2Array<CircleData> _circles = new();

    private int _vaoId;

    private readonly int[] _vboIds = new int[2];

    private int _programId;

    private readonly int _projectionUniform;

    private readonly int _pixelScaleUniform;

    private struct CircleData(Vector2 position, float radius, Color4 color)
    {
        public Vector2 Position = position;

        public float Radius = radius;

        public Color4 Color = color;
    }

    public GLCircles()
    {
        _programId = RenderHelper.CreateProgramFromFiles("Render/data/circle.vs", "Render/data/circle.fs");
        _projectionUniform = GL.GetUniformLocation(_programId, "projectionMatrix");
        _pixelScaleUniform = GL.GetUniformLocation(_programId, "pixelScale");
        int vertexAttribute = 0;
        int positionInstance = 1;
        int radiusInstance = 2;
        int colorInstance = 3;

        // Generate
        GL.GenVertexArrays(1, out _vaoId);
        GL.GenBuffers(2, _vboIds);

        GL.BindVertexArray(_vaoId);
        GL.EnableVertexAttribArray(vertexAttribute);
        GL.EnableVertexAttribArray(positionInstance);
        GL.EnableVertexAttribArray(radiusInstance);
        GL.EnableVertexAttribArray(colorInstance);

        // Vertex buffer for sinGL.e quad
        float a = 1.1f;
        Vec2[] vertices = [(-a, -a), (a, -a), (-a, a), (a, -a), (a, a), (-a, a)];

        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[0]);
        GL.BufferData(BufferTarget.ArrayBuffer, SizeCache<CircleData>.Size * vertices.Length, vertices, BufferUsageHint.StaticDraw);
        GL.VertexAttribPointer(vertexAttribute, 2, VertexAttribPointerType.Float, false, 0, 0);

        // Circle buffer
        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[1]);
        GL.BufferData(BufferTarget.ArrayBuffer, MaxCount * SizeCache<CircleData>.Size, 0, BufferUsageHint.DynamicDraw);

        GL.VertexAttribPointer(
            positionInstance,
            2,
            VertexAttribPointerType.Float,
            false,
            SizeCache<CircleData>.Size,
            0);
        GL.VertexAttribPointer(
            radiusInstance,
            1,
            VertexAttribPointerType.Float,
            false,
            SizeCache<CircleData>.Size,
            8);
        GL.VertexAttribPointer(
            colorInstance,
            4,
            VertexAttribPointerType.Float,
            true,
            SizeCache<CircleData>.Size,
            12);

        GL.VertexAttribDivisor(positionInstance, 1);
        GL.VertexAttribDivisor(radiusInstance, 1);
        GL.VertexAttribDivisor(colorInstance, 1);

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
            _vboIds[0] = 0;
            _vboIds[1] = 0;
        }

        if (_programId != 0)
        {
            GL.DeleteProgram(_programId);
            _programId = 0;
        }
    }

    public void AddCircle(Vector2 center, float radius, Color4 color)
    {
        _circles.Push(new(center, radius, color));
    }

    public void Flush()
    {
        int count = _circles.Count;
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

            GL.BufferSubData(BufferTarget.ArrayBuffer, 0, batchCount * SizeCache<CircleData>.Size, ref _circles[@base]);
            GL.DrawArraysInstanced(PrimitiveType.Triangles, 0, 6, batchCount);

            RenderHelper.CheckGLError();

            count -= MaxCount;
            @base += MaxCount;
        }

        GL.Disable(EnableCap.Blend);

        GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
        GL.BindVertexArray(0);
        GL.UseProgram(0);

        _circles.Clear();
    }
};