using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using Box2DSharp;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using Testbed.Abstractions;

namespace Testbed.Render;

public class GLSolidCapsules : IDisposable
{
    private const int MaxCount = 2048;

    private readonly B2Array<CapsuleData> _capsules = new();

    private int _vaoId;

    private readonly int[] _vboIds = new int[2];

    private int _programId;

    private readonly int _projectionUniform;

    private readonly int _pixelScaleUniform;

    public struct CapsuleData
    {
        public Transform Transform;

        public float Radius;

        public float Length;

        public Color4 Color;
    }

    public GLSolidCapsules()
    {
        _programId = RenderHelper.CreateProgramFromFiles("Render/data/solid_capsule.vs", "Render/data/solid_capsule.fs");

        _projectionUniform = GL.GetUniformLocation(_programId, "projectionMatrix");
        _pixelScaleUniform = GL.GetUniformLocation(_programId, "pixelScale");
        int vertexAttribute = 0;
        int transformInstance = 1;
        int radiusInstance = 2;
        int lengthInstance = 3;
        int colorInstance = 4;

        // Generate
        GL.GenVertexArrays(1, out _vaoId);
        GL.GenBuffers(2, _vboIds);

        GL.BindVertexArray(_vaoId);
        GL.EnableVertexAttribArray(vertexAttribute);
        GL.EnableVertexAttribArray(transformInstance);
        GL.EnableVertexAttribArray(radiusInstance);
        GL.EnableVertexAttribArray(lengthInstance);
        GL.EnableVertexAttribArray(colorInstance);

        // Vertex buffer for single quad
        float a = 1.1f;
        Vec2[] vertices = [(-a, -a), (a, -a), (-a, a), (a, -a), (a, a), (-a, a)];
        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[0]);
        GL.BufferData(BufferTarget.ArrayBuffer, SizeCache<Vec2>.Size * vertices.Length, vertices, BufferUsageHint.StaticDraw);
        GL.VertexAttribPointer(vertexAttribute, 2, VertexAttribPointerType.Float, false, 0, 0);

        // Capsule buffer
        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[1]);
        GL.BufferData(BufferTarget.ArrayBuffer, MaxCount * SizeCache<CapsuleData>.Size, 0, BufferUsageHint.DynamicDraw);

        GL.VertexAttribPointer(transformInstance, 4, VertexAttribPointerType.Float, false, SizeCache<CapsuleData>.Size, 0);
        GL.VertexAttribPointer(radiusInstance, 1, VertexAttribPointerType.Float, false, SizeCache<CapsuleData>.Size, 16);
        GL.VertexAttribPointer(lengthInstance, 1, VertexAttribPointerType.Float, false, SizeCache<CapsuleData>.Size, 20);
        GL.VertexAttribPointer(colorInstance, 4, VertexAttribPointerType.Float, true, SizeCache<CapsuleData>.Size, 24);

        // These divisors tell GL.sl how to distribute per instance data
        GL.VertexAttribDivisor(transformInstance, 1);
        GL.VertexAttribDivisor(radiusInstance, 1);
        GL.VertexAttribDivisor(lengthInstance, 1);
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

    public void AddCapsule(in Vec2 p1, in Vec2 p2, float radius, Color4 color)
    {
        Vec2 d = p2 - p1;
        float length = d.Length;
        if (length < 0.001f)
        {
            Debug.WriteLine("WARNING: sample app: capsule too short!\n");
            return;
        }

        Vec2 axis = (d.X / length, d.Y / length);
        Transform transform;
        transform.P = 0.5f * (p1 + p2);
        transform.Q.C = axis.X;
        transform.Q.S = axis.Y;

        _capsules.Push(
            new CapsuleData
            {
                Transform = transform,
                Radius = radius,
                Length = length,
                Color = color
            });
    }

    public void Flush()
    {
        var count = _capsules.Count;
        if (count == 0)
        {
            return;
        }

        GL.UseProgram(_programId);

        var proj = new float[16];
        Global.Camera.BuildProjectionMatrix(proj, 0.2f);

        GL.UniformMatrix4(_projectionUniform, 1, false, proj);
        GL.Uniform1(_pixelScaleUniform, Global.Camera.Height / Global.Camera.Zoom);

        GL.BindVertexArray(_vaoId);

        GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[1]);
        GL.Enable(EnableCap.Blend);
        GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

        var @base = 0;
        while (count > 0)
        {
            var batchCount = Math.Min(count, MaxCount);

            GL.BufferSubData(BufferTarget.ArrayBuffer, 0, batchCount * SizeCache<CapsuleData>.Size, ref _capsules[@base]);
            GL.DrawArraysInstanced(PrimitiveType.Triangles, 0, 6, batchCount);

            RenderHelper.CheckGLError();

            count -= MaxCount;
            @base += MaxCount;
        }

        GL.Disable(EnableCap.Blend);

        GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
        GL.BindVertexArray(0);
        GL.UseProgram(0);

        _capsules.Clear();
    }
}