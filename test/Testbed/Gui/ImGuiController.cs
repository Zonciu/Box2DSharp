using System;
using System.Runtime.CompilerServices;
using ImGuiNET;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using OpenTK.Windowing.GraphicsLibraryFramework;
using MouseButton = Testbed.Abstractions.MouseButton;

namespace Testbed.Gui
{
    /// <summary>
    /// A modified version of Veldrid.ImGui's ImGuiRenderer.
    /// Manages input for ImGui and handles rendering ImGui's DrawLists with Veldrid.
    /// </summary>
    public class ImGuiController : IDisposable
    {
        private bool _frameBegun;

        // Veldrid objects
        private int _vertexArray;

        private int _vertexBuffer;

        private int _vertexBufferSize;

        private int _indexBuffer;

        private int _indexBufferSize;

        private Texture _fontTexture;

        private Shader _shader;

        private int _windowWidth;

        private int _windowHeight;

        private readonly System.Numerics.Vector2 _scaleFactor = System.Numerics.Vector2.One;

        /// <summary>
        /// Constructs a new ImGuiController.
        /// </summary>
        public ImGuiController(int width, int height)
        {
            _windowWidth = width;
            _windowHeight = height;

            var context = ImGui.CreateContext();
            ImGui.SetCurrentContext(context);
            var io = ImGui.GetIO();

            io.Fonts.AddFontDefault();

            //io.BackendFlags |= ImGuiBackendFlags.RendererHasVtxOffset;
            io.BackendFlags |= ImGuiBackendFlags.HasMouseCursors; // We can honor GetMouseCursor() values (optional)
            io.BackendFlags |= ImGuiBackendFlags.HasSetMousePos;  // We can honor io.WantSetMousePos requests (optional, rarely used)
            CreateDeviceResources();
            SetPerFrameImGuiData(1f / 60f);

            ImGui.NewFrame();
            _frameBegun = true;
        }

        public void WindowResized(int width, int height)
        {
            _windowWidth = width;
            _windowHeight = height;
        }

        public void DestroyDeviceObjects()
        {
            Dispose();
        }

        public void CreateDeviceResources()
        {
            Util.CreateVertexArray("ImGui", out _vertexArray);

            _vertexBufferSize = 10000;
            _indexBufferSize = 2000;

            Util.CreateVertexBuffer("ImGui", out _vertexBuffer);
            Util.CreateElementBuffer("ImGui", out _indexBuffer);
            GL.NamedBufferData(_vertexBuffer, _vertexBufferSize, IntPtr.Zero, BufferUsageHint.DynamicDraw);
            GL.NamedBufferData(_indexBuffer, _indexBufferSize, IntPtr.Zero, BufferUsageHint.DynamicDraw);

            RecreateFontDeviceTexture();

            const string VertexSource = @"#version 330 core

uniform mat4 projection_matrix;

layout(location = 0) in vec2 in_position;
layout(location = 1) in vec2 in_texCoord;
layout(location = 2) in vec4 in_color;

out vec4 color;
out vec2 texCoord;

void main()
{
    gl_Position = projection_matrix * vec4(in_position, 0, 1);
    color = in_color;
    texCoord = in_texCoord;
}";
            const string FragmentSource = @"#version 330 core

uniform sampler2D in_fontTexture;

in vec4 color;
in vec2 texCoord;

out vec4 outputColor;

void main()
{
    outputColor = color * texture(in_fontTexture, texCoord);
}";

            _shader = new Shader("ImGui", VertexSource, FragmentSource);

            GL.VertexArrayVertexBuffer(_vertexArray, 0, _vertexBuffer, IntPtr.Zero, Unsafe.SizeOf<ImDrawVert>());
            GL.VertexArrayElementBuffer(_vertexArray, _indexBuffer);

            GL.EnableVertexArrayAttrib(_vertexArray, 0);
            GL.VertexArrayAttribBinding(_vertexArray, 0, 0);
            GL.VertexArrayAttribFormat(_vertexArray, 0, 2, VertexAttribType.Float, false, 0);

            GL.EnableVertexArrayAttrib(_vertexArray, 1);
            GL.VertexArrayAttribBinding(_vertexArray, 1, 0);
            GL.VertexArrayAttribFormat(_vertexArray, 1, 2, VertexAttribType.Float, false, 8);

            GL.EnableVertexArrayAttrib(_vertexArray, 2);
            GL.VertexArrayAttribBinding(_vertexArray, 2, 0);
            GL.VertexArrayAttribFormat(_vertexArray, 2, 4, VertexAttribType.UnsignedByte, true, 16);

            Util.CheckGLError("End of ImGui setup");
        }

        /// <summary>
        /// Recreates the device texture used to render text.
        /// </summary>
        public void RecreateFontDeviceTexture()
        {
            var io = ImGui.GetIO();
            io.Fonts.GetTexDataAsRGBA32(out IntPtr pixels, out var width, out var height, out var bytesPerPixel);

            _fontTexture = new Texture("ImGui Text Atlas", width, height, pixels);
            _fontTexture.SetMagFilter(TextureMagFilter.Linear);
            _fontTexture.SetMinFilter(TextureMinFilter.Linear);

            io.Fonts.SetTexID((IntPtr)_fontTexture.GLTexture);

            io.Fonts.ClearTexData();
        }

        /// <summary>
        /// Renders the ImGui draw list data.
        /// This method requires a GraphicsDevice because it may create new DeviceBuffers if the size of vertex
        /// or index data has increased beyond the capacity of the existing buffers.
        /// A CommandList is needed to submit drawing and resource update commands.
        /// </summary>
        public void Render()
        {
            if (!_frameBegun)
            {
                return;
            }

            _frameBegun = false;
            ImGui.Render();
            RenderImDrawData(ImGui.GetDrawData());
        }

        /// <summary>
        /// Updates ImGui input and IO configuration state.
        /// </summary>
        public void Update(Game wnd, float deltaSeconds)
        {
            if (_frameBegun)
            {
                ImGui.Render();
            }

            SetPerFrameImGuiData(deltaSeconds);
            UpdateImGuiInput(wnd);

            _frameBegun = true;
            ImGui.NewFrame();
        }

        /// <summary>
        /// Sets per-frame data based on the associated window.
        /// This is called by Update(float).
        /// </summary>
        private void SetPerFrameImGuiData(float deltaSeconds)
        {
            var io = ImGui.GetIO();
            io.DisplaySize = new System.Numerics.Vector2(
                _windowWidth / _scaleFactor.X,
                _windowHeight / _scaleFactor.Y);
            io.DisplayFramebufferScale = _scaleFactor;
            io.DeltaTime = deltaSeconds; // DeltaTime is in seconds.
        }

        private static void UpdateImGuiInput(Game wnd)
        {
            var io = ImGui.GetIO();

            var keyboardState = wnd.KeyboardState;

            io.MouseDown[0] = wnd.IsMouseButtonDown(Input.GetMouse(MouseButton.Left));
            io.MouseDown[1] = wnd.IsMouseButtonDown(Input.GetMouse(MouseButton.Right));
            io.MouseDown[2] = wnd.IsMouseButtonDown(Input.GetMouse(MouseButton.Middle));

            io.MousePos = new System.Numerics.Vector2(wnd.MousePosition.X, wnd.MousePosition.Y);
            io.MouseWheel = wnd.Scroll.Y;
            io.MouseWheelH = wnd.Scroll.X;

            io.KeyCtrl = keyboardState.IsKeyDown(Keys.LeftControl) || keyboardState.IsKeyDown(Keys.RightControl);
            io.KeyAlt = keyboardState.IsKeyDown(Keys.LeftAlt) || keyboardState.IsKeyDown(Keys.RightAlt);
            io.KeyShift = keyboardState.IsKeyDown(Keys.LeftShift) || keyboardState.IsKeyDown(Keys.RightShift);
            io.KeySuper = keyboardState.IsKeyDown(Keys.LeftSuper) || keyboardState.IsKeyDown(Keys.RightSuper);
        }

        private void RenderImDrawData(ImDrawDataPtr drawData)
        {
            if (drawData.CmdListsCount == 0)
            {
                return;
            }

            for (var i = 0; i < drawData.CmdListsCount; i++)
            {
                var cmdList = drawData.CmdLists[i];

                var vertexSize = cmdList.VtxBuffer.Size * Unsafe.SizeOf<ImDrawVert>();
                if (vertexSize > _vertexBufferSize)
                {
                    var newSize = (int)Math.Max(_vertexBufferSize * 1.5f, vertexSize);
                    GL.NamedBufferData(_vertexBuffer, newSize, IntPtr.Zero, BufferUsageHint.DynamicDraw);
                    _vertexBufferSize = newSize;
                }

                var indexSize = cmdList.IdxBuffer.Size * sizeof(ushort);
                if (indexSize <= _indexBufferSize)
                {
                    continue;
                }

                {
                    var newSize = (int)Math.Max(_indexBufferSize * 1.5f, indexSize);
                    GL.NamedBufferData(_indexBuffer, newSize, IntPtr.Zero, BufferUsageHint.DynamicDraw);
                    _indexBufferSize = newSize;
                }
            }

            // Setup orthographic projection matrix into our constant buffer
            var io = ImGui.GetIO();
            var mvp = Matrix4.CreateOrthographicOffCenter(
                0.0f,
                io.DisplaySize.X,
                io.DisplaySize.Y,
                0.0f,
                -1.0f,
                1.0f);

            _shader.UseShader();
            GL.UniformMatrix4(_shader.GetUniformLocation("projection_matrix"), false, ref mvp);
            GL.Uniform1(_shader.GetUniformLocation("in_fontTexture"), 0);
            Util.CheckGLError("Projection");

            GL.BindVertexArray(_vertexArray);
            Util.CheckGLError("VAO");

            drawData.ScaleClipRects(io.DisplayFramebufferScale);

            GL.Enable(EnableCap.Blend);
            GL.Enable(EnableCap.ScissorTest);
            GL.BlendEquation(BlendEquationMode.FuncAdd);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            GL.Disable(EnableCap.CullFace);
            GL.Disable(EnableCap.DepthTest);
            Util.CheckGLError($"Render state");

            // Render command lists
            for (var n = 0; n < drawData.CmdListsCount; n++)
            {
                var cmdList = drawData.CmdLists[n];

                GL.NamedBufferSubData(_vertexBuffer, IntPtr.Zero, cmdList.VtxBuffer.Size * Unsafe.SizeOf<ImDrawVert>(), cmdList.VtxBuffer.Data);
                Util.CheckGLError($"Data Vert {n}");

                GL.NamedBufferSubData(_indexBuffer, IntPtr.Zero, cmdList.IdxBuffer.Size * sizeof(ushort), cmdList.IdxBuffer.Data);
                Util.CheckGLError($"Data Idx {n}");

                var vtxOffset = 0;
                var idxOffset = 0;

                for (var cmd_i = 0; cmd_i < cmdList.CmdBuffer.Size; cmd_i++)
                {
                    var pcmd = cmdList.CmdBuffer[cmd_i];
                    if (pcmd.UserCallback != IntPtr.Zero)
                    {
                        throw new NotImplementedException();
                    }
                    else
                    {
                        GL.ActiveTexture(TextureUnit.Texture0);
                        GL.BindTexture(TextureTarget.Texture2D, (int)pcmd.TextureId);
                        Util.CheckGLError("Texture");

                        // We do _windowHeight - (int)clip.W instead of (int)clip.Y because gl has flipped Y when it comes to these coordinates
                        var clip = pcmd.ClipRect;
                        GL.Scissor((int)clip.X, _windowHeight - (int)clip.W, (int)(clip.Z - clip.X), (int)(clip.W - clip.Y));
                        Util.CheckGLError("Scissor");

                        if ((io.BackendFlags & ImGuiBackendFlags.RendererHasVtxOffset) != 0)
                        {
                            GL.DrawElementsBaseVertex(
                                PrimitiveType.Triangles,
                                (int)pcmd.ElemCount,
                                DrawElementsType.UnsignedShort,
                                (IntPtr)(idxOffset * sizeof(ushort)),
                                vtxOffset);
                        }
                        else
                        {
                            GL.DrawElements(BeginMode.Triangles, (int)pcmd.ElemCount, DrawElementsType.UnsignedShort, (int)pcmd.IdxOffset * sizeof(ushort));
                        }

                        Util.CheckGLError("Draw");
                    }

                    idxOffset += (int)pcmd.ElemCount;
                }

                vtxOffset += cmdList.VtxBuffer.Size;
            }

            GL.ActiveTexture(TextureUnit.Texture0);
            GL.BindTexture(TextureTarget.Texture2D, 0);

            GL.Disable(EnableCap.Blend);
            GL.Disable(EnableCap.ScissorTest);

            GL.BindVertexArray(0);
        }

        /// <summary>
        /// Frees all graphics resources used by the renderer.
        /// </summary>
        public void Dispose()
        {
            _fontTexture.Dispose();
            _shader.Dispose();
        }
    }
}