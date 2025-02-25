using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Continuous;

[TestInherit]
public class SkinnyBox : Samples.Continuous.SkinnyBox
{
    public SkinnyBox(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        base.UpdateUI();

        float height = 110.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(140.0f, height));

        ImGui.Begin("Skinny Box", ImGuiWindowFlags.NoResize);

        ImGui.Checkbox("Capsule", ref m_capsule);

        if (ImGui.Button("Launch"))
        {
            Launch();
        }

        ImGui.Checkbox("Auto Test", ref m_autoTest);

        ImGui.End();
    }
}