using System;
using System.Diagnostics;
using OpenToolkit.Graphics.OpenGL4;

namespace Testbed.Basics
{
    public static class Render
    {
        public static void CheckGLError()
        {
            var errCode = GL.GetError();
            if (errCode == ErrorCode.NoError)
            {
                return;
            }

            Console.WriteLine($"OpenGL error = {errCode}");
            Debug.Assert(false);
        }

        public static void PrintLog(int obj)
        {
            string log = null;
            if (GL.IsShader(obj))
            {
                GL.GetShaderInfoLog(obj, out log);
            }
            else if (GL.IsProgram(obj))
            {
                GL.GetProgramInfoLog(obj, out log);
            }

            Console.WriteLine(!string.IsNullOrWhiteSpace(log) ? log : "printlog: Not a shader or a program");
        }

        public static int CreateShaderFromString(string source, ShaderType type)
        {
            var res = GL.CreateShader(type);

            GL.ShaderSource(res, source);
            GL.CompileShader(res);
            var error = GL.GetShaderInfoLog(res);
            if (!string.IsNullOrWhiteSpace(error))
            {
                Console.WriteLine($"Error compiling shader {res} of type {type}: {error}");
                PrintLog(res);
                GL.DeleteShader(res);
                return 0;
            }

            return res;
        }

        // 
        public static int CreateShaderProgram(string vs, string fs)
        {
            var vsId = CreateShaderFromString(vs, ShaderType.VertexShader);
            var fsId = CreateShaderFromString(fs, ShaderType.FragmentShader);
            Debug.Assert(vsId != 0 && fsId != 0);

            var programId = GL.CreateProgram();
            GL.AttachShader(programId, vsId);
            GL.AttachShader(programId, fsId);
            GL.BindFragDataLocation(programId, 0, "color");
            GL.LinkProgram(programId);

            GL.DeleteShader(vsId);
            GL.DeleteShader(fsId);

            GL.GetProgram(programId, GetProgramParameterName.LinkStatus, out var status);
            Debug.Assert(status != (int)All.False);

            return programId;
        }
    }
}