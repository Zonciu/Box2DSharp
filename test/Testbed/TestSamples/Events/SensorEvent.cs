using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Events;

[TestInherit]
public class SensorEvent : Samples.Events.SensorEvent
{
    public SensorEvent(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        base.UpdateUI();
        float height = 90.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(140.0f, height));

        ImGui.Begin("Sensor Event", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

        if (ImGui.RadioButton("donut", Type == Donut))
        {
            Clear();
            Type = Donut;
        }

        if (ImGui.RadioButton("human", Type == Human))
        {
            Clear();
            Type = Human;
        }

        ImGui.End();
    }
}