﻿using System;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using Testbed.Abstractions;
using Vector2 = System.Numerics.Vector2;

namespace Testbed.Render
{
    public class GLRenderTriangles : IDisposable
    {
        private const int MaxVertices = 3 * 2048;

        private readonly Vector2[] _vertices = new Vector2[MaxVertices];

        private readonly Color4[] _colors = new Color4[MaxVertices];

        private int _count;

        private int _vaoId;

        private readonly int[] _vboIds = new int[2];

        private int _programId;

        private int _projectionUniform;

        private int _vertexAttribute;

        private int _colorAttribute;

        public GLRenderTriangles()
        {
            const string VertexShaderSource =
                @"#version 330
            uniform mat4 projectionMatrix;
            layout(location = 0) in vec2 v_position;
            layout(location = 1) in vec4 v_color;
            out vec4 f_color;
            void main(void)
            {
            	f_color = v_color;
            	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);
            }";

            const string FragmentShaderSource =
                @"#version 330
            in vec4 f_color;
            out vec4 color;
            void main(void)
            {
            	color = f_color;
            }";

            _programId = RenderHelper.CreateProgramFromStrings(VertexShaderSource, FragmentShaderSource);
            _projectionUniform = GL.GetUniformLocation(_programId, "projectionMatrix");
            _vertexAttribute = 0;
            _colorAttribute = 1;

            // Generate
            _vaoId = GL.GenVertexArray();
            GL.GenBuffers(2, _vboIds);

            GL.BindVertexArray(_vaoId);
            GL.EnableVertexAttribArray(_vertexAttribute);
            GL.EnableVertexAttribArray(_colorAttribute);

            // Vertex buffer
            GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[0]);
            GL.VertexAttribPointer(_vertexAttribute, 2, VertexAttribPointerType.Float, false, 0, 0);
            GL.BufferData(BufferTarget.ArrayBuffer, SizeCache<Vector2>.Size * MaxVertices, _vertices, BufferUsageHint.DynamicDraw);

            GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[1]);
            GL.VertexAttribPointer(_colorAttribute, 4, VertexAttribPointerType.Float, false, 0, 0);
            GL.BufferData(BufferTarget.ArrayBuffer, SizeCache<Color4>.Size * MaxVertices, _colors, BufferUsageHint.DynamicDraw);

            RenderHelper.CheckGLError();

            // Cleanup
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.BindVertexArray(0);

            _count = 0;
        }

        public void Dispose()
        {
            if (_vaoId != 0)
            {
                GL.DeleteVertexArray(_vaoId);
                GL.DeleteBuffers(2, _vboIds);
                _vaoId = 0;
            }

            if (_programId != 0)
            {
                GL.DeleteProgram(_programId);
                _programId = 0;
            }
        }

        public void Vertex(Vector2 v, Color4 c)
        {
            if (_count == MaxVertices)
            {
                Flush();
            }

            _vertices[_count] = v;
            _colors[_count] = c;
            ++_count;
        }

        public void Flush()
        {
            if (_count == 0)
            {
                return;
            }

            GL.UseProgram(_programId);

            var proj = new float[16];
            Global.Camera.BuildProjectionMatrix(proj, 0.2f);

            GL.UniformMatrix4(_projectionUniform, 1, false, proj);

            GL.BindVertexArray(_vaoId);

            GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[0]);
            GL.BufferSubData(BufferTarget.ArrayBuffer, (IntPtr)0, _count * SizeCache<Vector2>.Size, _vertices);
            RenderHelper.CheckGLError();
            GL.BindBuffer(BufferTarget.ArrayBuffer, _vboIds[1]);
            GL.BufferSubData(BufferTarget.ArrayBuffer, (IntPtr)0, _count * SizeCache<Color4>.Size, _colors);
            RenderHelper.CheckGLError();
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            GL.DrawArrays(PrimitiveType.Triangles, 0, _count);
            GL.Disable(EnableCap.Blend);

            RenderHelper.CheckGLError();

            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.BindVertexArray(0);
            GL.UseProgram(0);

            _count = 0;
        }
    }
}