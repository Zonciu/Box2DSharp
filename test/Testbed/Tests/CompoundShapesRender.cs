using System.Numerics;
using ImGuiNET;
using Testbed.TestCases;

namespace Testbed.Tests
{
    [TestInherit]
    public class CompoundShapesRender : CompoundShapes
    {
        /// <inheritdoc />
        protected override void OnRender()
        {
            ImGui.SetNextWindowPos(new Vector2(10.0f, 100.0f));
            ImGui.SetNextWindowSize(new Vector2(200.0f, 100.0f));
            ImGui.Begin("Controls", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

            if (ImGui.Button("Spawn"))
            {
                base.Spawn();
            }

            ImGui.End();
            base.OnRender();
        }
    }
}