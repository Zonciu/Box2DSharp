using System;
using Box2DSharp;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using Testbed.Abstractions;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.Render;

public class GLPoints : IDisposable
{
    private const int MaxCount = 2048;

    private readonly B2Array<PointData> _points = new(MaxCount);

    private int _vaoId;

    private int _vboId;

    private int _programId;

    private readonly int _projectionUniform;

    private struct PointData(Vec2 position, float size, Color4 color)
    {
        public Vec2 Position = position;

        public float Size = size;

        public Color4 Color = color;
    }

    public GLPoints()
    {
        const string vs = """
                          #version 330
                          uniform mat4 projectionMatrix;
                          layout(location = 0) in vec2 v_position;
                          layout(location = 1) in float v_size;
                          layout(location = 2) in vec4 v_color;
                          out vec4 f_color;
                          void main(void)
                          {
                          	f_color = v_color;
                          	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);
                          	gl_PointSize = v_size;
                          }
                          """;

        const string fs = """
                          #version 330
                          in vec4 f_color;
                          out vec4 color;
                          void main(void)
                          {
                          	color = f_color;
                          }
                          """;

        _programId = RenderHelper.CreateProgramFromStrings(vs, fs);
        _projectionUniform = GL.GetUniformLocation(_programId, "projectionMatrix");
        int vertexAttribute = 0;
        int sizeAttribute = 1;
        int colorAttribute = 2;

        // Generate
        GL.GenVertexArrays(1, out _vaoId);
        GL.GenBuffers(1, out _vboId);

        GL.BindVertexArray(_vaoId);
        GL.EnableVertexAttribArray(vertexAttribute);
        GL.EnableVertexAttribArray(sizeAttribute);
        GL.EnableVertexAttribArray(colorAttribute);

        // Vertex buffer
        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboId);
        GL.BufferData(BufferTarget.ArrayBuffer, MaxCount * SizeCache<PointData>.Size, 0, BufferUsageHint.DynamicDraw);

        GL.VertexAttribPointer(
            vertexAttribute,
            2,
            VertexAttribPointerType.Float,
            false,
            SizeCache<PointData>.Size,
            0);
        GL.VertexAttribPointer(
            sizeAttribute,
            1,
            VertexAttribPointerType.Float,
            false,
            SizeCache<PointData>.Size,
            8);

        // color will get automatically expanded to floats in the shader
        GL.VertexAttribPointer(
            colorAttribute,
            4,
            VertexAttribPointerType.Float,
            true,
            SizeCache<PointData>.Size,
            12);

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
            GL.DeleteBuffers(1, ref _vboId);
            _vaoId = 0;
            _vboId = 0;
        }

        if (_programId != 0)
        {
            GL.DeleteProgram(_programId);
            _programId = 0;
        }
    }

    // todo instead of flushing, keep a growable array of data
    // this will prevent sorting problems.

    public void AddPoint(Vector2 v, float size, Color4 c)
    {
        _points.Push(new(v, size, c));
    }

    public void Flush()
    {
        int count = (int)_points.Count;
        if (count == 0)
        {
            return;
        }

        GL.UseProgram(_programId);

        float[] proj = new float[16];
        Global.Camera.BuildProjectionMatrix(proj, 0.0f);

        GL.UniformMatrix4(_projectionUniform, 1, false, proj);
        GL.BindVertexArray(_vaoId);

        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboId);
        GL.Enable(EnableCap.ProgramPointSize);

        int @base = 0;
        while (count > 0)
        {
            int batchCount = Math.Min(count, MaxCount);
            GL.BufferSubData(BufferTarget.ArrayBuffer, 0, batchCount * SizeCache<PointData>.Size, ref _points[@base]);
            GL.DrawArrays(PrimitiveType.Points, 0, batchCount);

            RenderHelper.CheckGLError();

            count -= MaxCount;
            @base += MaxCount;
        }

        GL.Disable(EnableCap.ProgramPointSize);
        GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
        GL.BindVertexArray(0);
        GL.UseProgram(0);

        _points.Clear();
    }
};