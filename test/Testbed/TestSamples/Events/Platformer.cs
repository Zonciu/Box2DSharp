using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Events;

[TestInherit]
public class Platformer : Samples.Events.Platformer
{
    public Platformer(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        base.UpdateUI();
        float height = 100.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Platformer", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

        ImGui.SliderFloat("force", ref m_force, 0.0f, 50.0f, "%.1f");
        ImGui.SliderFloat("impulse", ref m_impulse, 0.0f, 50.0f, "%.1f");

        ImGui.End();

        DrawString("Movement: A/D/Space");

        DrawString($"Can jump = {_canJump}");
    }
}