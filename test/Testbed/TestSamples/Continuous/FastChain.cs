using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Continuous;

[TestInherit]
public class FastChain : Samples.Continuous.FastChain
{
    public FastChain(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        base.UpdateUI();

        float height = 70.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Fast Chain", ImGuiWindowFlags.NoResize);

        if (ImGui.Button("Launch"))
        {
            Launch();
        }

        ImGui.End();
    }
}