using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Events;

[TestInherit]
public class ContactEvent : Samples.Events.ContactEvent
{
    public ContactEvent(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        base.UpdateUI();
        float height = 60.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Contact Event", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

        ImGui.SliderFloat("force", ref m_force, 100.0f, 500.0f, "%.1f");

        ImGui.End();

        DrawString("move using WASD");
    }
}