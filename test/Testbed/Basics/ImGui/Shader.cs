using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using OpenToolkit.Graphics.OpenGL4;

namespace Testbed.Basics.ImGui
{
    struct UniformFieldInfo
    {
        public int Location;

        public string Name;

        public int Size;

        public ActiveUniformType Type;
    }

    class Shader
    {
        public readonly string Name;

        public int Program { get; private set; }

        private readonly Dictionary<string, int> _uniformToLocation = new Dictionary<string, int>();

        private bool _initialized = false;

        public Shader(string name, string vertexShader, string fragmentShader)
        {
            Name = name;
            (ShaderType Type, string Path)[] files =
            {
                (ShaderType.VertexShader, vertexShader),
                (ShaderType.FragmentShader, fragmentShader),
            };
            Program = CreateProgram(name, files);
        }

        public void UseShader()
        {
            GL.UseProgram(Program);
        }

        public void Dispose()
        {
            if (_initialized)
            {
                GL.DeleteProgram(Program);
                _initialized = false;
            }
        }

        public UniformFieldInfo[] GetUniforms()
        {
            GL.GetProgram(Program, GetProgramParameterName.ActiveUniforms, out var unifromCount);

            var uniforms = new UniformFieldInfo[unifromCount];

            for (var i = 0; i < unifromCount; i++)
            {
                var name = GL.GetActiveUniform(Program, i, out var size, out var type);

                uniforms[i] = new UniformFieldInfo
                {
                    Location = GetUniformLocation(name),
                    Name = name,
                    Size = size,
                    Type = type
                };
                ;
            }

            return uniforms;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetUniformLocation(string uniform)
        {
            if (_uniformToLocation.TryGetValue(uniform, out var location) == false)
            {
                location = GL.GetUniformLocation(Program, uniform);
                _uniformToLocation.Add(uniform, location);

                if (location == -1)
                {
                    Debug.Print($"The uniform '{uniform}' does not exist in the shader '{Name}'!");
                }
            }

            return location;
        }

        private int CreateProgram(string name, params (ShaderType Type, string source)[] shaderPaths)
        {
            Util.CreateProgram(name, out var program);

            var shaders = new int[shaderPaths.Length];
            for (var i = 0; i < shaderPaths.Length; i++)
            {
                shaders[i] = CompileShader(name, shaderPaths[i].Type, shaderPaths[i].source);
            }

            foreach (var shader in shaders)
            {
                GL.AttachShader(program, shader);
            }

            GL.LinkProgram(program);

            GL.GetProgram(program, GetProgramParameterName.LinkStatus, out int Success);
            if (Success == 0)
            {
                var info = GL.GetProgramInfoLog(program);
                Debug.WriteLine($"GL.LinkProgram had info log [{name}]:\n{info}");
            }

            foreach (var shader in shaders)
            {
                GL.DetachShader(program, shader);
                GL.DeleteShader(shader);
            }

            _initialized = true;

            return program;
        }

        private int CompileShader(string name, ShaderType type, string source)
        {
            Util.CreateShader(type, name, out var shader);
            GL.ShaderSource(shader, source);
            GL.CompileShader(shader);

            GL.GetShader(shader, ShaderParameter.CompileStatus, out var success);
            if (success == 0)
            {
                var info = GL.GetShaderInfoLog(shader);
                Debug.WriteLine($"GL.CompileShader for shader '{Name}' [{type}] had info log:\n{info}");
            }

            return shader;
        }
    }
}