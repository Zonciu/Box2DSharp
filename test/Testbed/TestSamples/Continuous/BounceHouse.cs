using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Continuous;

[TestInherit]
public class BounceHouse : Samples.Continuous.BounceHouse
{
    public BounceHouse(Settings settings)
        : base(settings)
    { }

    private readonly string[] _shapeTypes = ["Circle", "Capsule", "Box"];

    public override void UpdateUI()
    {
        base.UpdateUI();

        float height = 100.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Bounce House", ImGuiWindowFlags.NoResize);

        if (ImGui.Combo("Shape", ref m_shapeType, _shapeTypes, _shapeTypes.Length))
        {
            Launch();
        }

        if (ImGui.Checkbox("hit events", ref m_enableHitEvents))
        {
            Body.EnableHitEvents(m_bodyId, m_enableHitEvents);
        }

        ImGui.End();
    }
}