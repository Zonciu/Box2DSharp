using System;
using System.Diagnostics;
using Box2DSharp;
using OpenTK.Graphics.OpenGL;
using Testbed.Abstractions;

namespace Testbed.Render;

public class GLBackground : IDisposable
{
    private int _vaoId;

    private int _vboId;

    private int _programId;

    private readonly int _timeUniform;

    private readonly int _resolutionUniform;

    private readonly int _baseColorUniform;

    public GLBackground()
    {
        _programId = RenderHelper.CreateProgramFromFiles("Render/data/background.vs", "Render/data/background.fs");
        _timeUniform = GL.GetUniformLocation(_programId, "time");
        _resolutionUniform = GL.GetUniformLocation(_programId, "resolution");
        _baseColorUniform = GL.GetUniformLocation(_programId, "baseColor");
        int vertexAttribute = 0;

        // Generate
        GL.GenVertexArrays(1, out _vaoId);
        GL.GenBuffers(1, out _vboId);

        GL.BindVertexArray(_vaoId);
        GL.EnableVertexAttribArray(vertexAttribute);

        // SinGL.e quad
        Vec2[] vertices = [(-1.0f, 1.0f), (-1.0f, -1.0f), (1.0f, 1.0f), (1.0f, -1.0f)];
        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboId);
        GL.BufferData(BufferTarget.ArrayBuffer, SizeCache<Vec2>.Size * vertices.Length, vertices, BufferUsageHint.StaticDraw);
        GL.VertexAttribPointer(vertexAttribute, 2, VertexAttribPointerType.Float, false, 0, 0);

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

    private readonly long _startTime = Stopwatch.GetTimestamp();

    public void Draw()
    {
        GL.UseProgram(_programId);
        var time = (float)Stopwatch.GetElapsedTime(_startTime).TotalSeconds;
        GL.Uniform1(_timeUniform, time);
        GL.Uniform2(_resolutionUniform, (float)Global.Camera.Width, (float)Global.Camera.Height);
        GL.Uniform3(_baseColorUniform, 0.2f, 0.2f, 0.2f);
        GL.BindVertexArray(_vaoId);
        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboId);
        GL.DrawArrays(PrimitiveType.TriangleStrip, 0, 4);
        GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
        GL.BindVertexArray(0);
        GL.UseProgram(0);
    }
}