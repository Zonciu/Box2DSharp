using ImGuiNET;
using Testbed.TestCases;
using UnityEngine;

namespace Box2DSharp.Testbed.Unity.Tests
{
    [TestInherit]
    public class RayCastRender : RayCast
    {
        /// <inheritdoc />
        protected override void OnRender()
        {
            ImGui.SetNextWindowPos(new Vector2(10.0f, 100.0f));
            ImGui.SetNextWindowSize(new Vector2(210.0f, 285.0f));
            ImGui.Begin("Ray-cast Controls", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

            if (ImGui.Button("Shape 1"))
            {
                Create(0);
            }

            if (ImGui.Button("Shape 2"))
            {
                Create(1);
            }

            if (ImGui.Button("Shape 3"))
            {
                Create(2);
            }

            if (ImGui.Button("Shape 4"))
            {
                Create(3);
            }

            if (ImGui.Button("Shape 5"))
            {
                Create(4);
            }

            if (ImGui.Button("Shape 6"))
            {
                Create(5);
            }

            if (ImGui.Button("Destroy Shape"))
            {
                DestroyBody();
            }

            var mode = (int)_mode;
            ImGui.RadioButton("Any", ref mode, (int)Mode.Any);
            ImGui.RadioButton("Closest", ref mode, (int)Mode.Closest);
            ImGui.RadioButton("Multiple", ref mode, (int)Mode.Multiple);
            _mode = (Mode)mode;
            ImGui.SliderFloat("Angle", ref _degrees, 0.0f, 360.0f, "%.0f");
            ImGui.End();
            base.OnRender();
        }
    }
}