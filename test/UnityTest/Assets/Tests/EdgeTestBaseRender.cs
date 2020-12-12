using ImGuiNET;
using Testbed.TestCases;
using Vector2 = UnityEngine.Vector2;

namespace Box2DSharp.Testbed.Unity.Tests
{
    [TestInherit]
    public class EdgeTestBaseRender : EdgeTestBase
    {
        /// <inheritdoc />
        protected override void OnRender()
        {
            ImGui.SetNextWindowPos(new Vector2(10.0f, 100.0f));
            ImGui.SetNextWindowSize(new Vector2(200.0f, 100.0f));
            ImGui.Begin("Custom Controls", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

            if (ImGui.RadioButton("Boxes", Boxes == true))
            {
                CreateBoxes();
                Boxes = true;
            }

            if (ImGui.RadioButton("Circles", Boxes == false))
            {
                CreateCircles();
                Boxes = false;
            }

            ImGui.End();
            base.OnRender();
        }
    }
}