using System;
using Box2DSharp;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using Testbed.Abstractions;

namespace Testbed.Render;

public class GLSolidCircles : IDisposable
{
    private const int MaxCount = 2048;

    private readonly B2Array<SolidCircleData> _circles = new();

    private int _vaoId;

    private readonly int[] _vboIds = new int[2];

    private int _programId;

    private readonly int _projectionUniform;

    private readonly int _pixelScaleUniform;

    private struct SolidCircleData
    {
        public Transform Transform;

        public float Radius;

        public Color4 Color;
    }

    // Draws SDF circles using quad instancing. Apparently instancing of quads can be slow on older GPUs.
    // https://www.reddit.com/r/openGL./comments/q7yikr/how_to_draw_several_quads_through_instancing/
    // https://www.g-truc.net/post-0666.html

    public GLSolidCircles()
    {
        _programId = RenderHelper.CreateProgramFromFiles("Render/data/solid_circle.vs", "Render/data/solid_circle.fs");
        _projectionUniform = GL.GetUniformLocation(_programId, "projectionMatrix");
        _pixelScaleUniform = GL.GetUniformLocation(_programId, "pixelScale");

        // Generate
        GL.GenVertexArrays(1, out _vaoId);
        GL.GenBuffers(2, _vboIds);

        GL.BindVertexArray(_vaoId);

        int vertexAttribute = 0;
        int transformInstance = 1;
        int radiusInstance = 2;
        int colorInstance = 3;
        GL.EnableVertexAttribArray(vertexAttribute);
        GL.EnableVertexAttribArray(transformInstance);
        GL.EnableVertexAttribArray(radiusInstance);
        GL.EnableVertexAttribArray(colorInstance);

        // Vertex buffer for sinGL.e quad
        float a = 1.1f;
        Vec2[] vertices = [(-a, -a), (a, -a), (-a, a), (a, -a), (a, a), (-a, a)];
        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[0]);
        GL.BufferData(BufferTarget.ArrayBuffer, SizeCache<Vec2>.Size * vertices.Length, vertices, BufferUsageHint.StaticDraw);
        GL.VertexAttribPointer(vertexAttribute, 2, VertexAttribPointerType.Float, false, 0, 0);

        // Circle buffer
        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[1]);
        GL.BufferData(BufferTarget.ArrayBuffer, MaxCount * SizeCache<SolidCircleData>.Size, 0, BufferUsageHint.DynamicDraw);

        GL.VertexAttribPointer(
            transformInstance,
            4,
            VertexAttribPointerType.Float,
            false,
            SizeCache<SolidCircleData>.Size,
            0);
        GL.VertexAttribPointer(
            radiusInstance,
            1,
            VertexAttribPointerType.Float,
            false,
            SizeCache<SolidCircleData>.Size,
            16);
        GL.VertexAttribPointer(
            colorInstance,
            4,
            VertexAttribPointerType.Float,
            true,
            SizeCache<SolidCircleData>.Size,
            20);

        GL.VertexAttribDivisor(transformInstance, 1);
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

    public void AddCircle(in Transform transform, float radius, Color4 color)
    {
        _circles.Push(
            new SolidCircleData
            {
                Transform = transform,
                Radius = radius,
                Color = color
            });
    }

    public void Flush()
    {
        int count = _circles.Count;
        if (count == 0)
        {
            return;
        }

        GL.UseProgram(_programId);

        float[] proj = new float[16];
        Global.Camera.BuildProjectionMatrix(proj, 0.2f);

        GL.UniformMatrix4(_projectionUniform, 1, false, proj);
        GL.Uniform1(_pixelScaleUniform, Global.Camera.Height / Global.Camera.Zoom);

        GL.BindVertexArray(_vaoId);

        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[1]);
        GL.Enable(EnableCap.Blend);
        GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

        int @base = 0;
        while (count > 0)
        {
            int batchCount = Math.Min(count, MaxCount);

            GL.BufferSubData(BufferTarget.ArrayBuffer, 0, batchCount * SizeCache<SolidCircleData>.Size, ref _circles[@base]);
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
}