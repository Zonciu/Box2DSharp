using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Stacking;

[TestInherit]
public class Cliff(Settings settings) : Samples.Stacking.Cliff(settings)
{
    public override void UpdateUI()
    {
        float height = 60.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(160.0f, height));

        ImGui.Begin("Cliff", ImGuiWindowFlags.NoResize);

        if (ImGui.Button("Flip"))
        {
            Flip = !Flip;
            CreateBodies();
        }

        ImGui.End();
    }
}