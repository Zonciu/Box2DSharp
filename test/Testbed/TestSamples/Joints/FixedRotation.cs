using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

[TestInherit]
public class FixedRotation : Samples.Joints.FixedRotation
{
    public FixedRotation(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 60.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(180.0f, height));

        ImGui.Begin("Fixed Rotation", ImGuiWindowFlags.NoResize);

        if (ImGui.Checkbox("Fixed Rotation", ref m_fixedRotation))
        {
            for (int i = 0; i < e_count; ++i)
            {
                Body.SetFixedRotation(m_bodyIds[i], m_fixedRotation);
            }
        }

        ImGui.End();
    }
}