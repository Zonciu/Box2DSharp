using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Events;

[TestInherit]
public class BodyMove : Samples.Events.BodyMove
{
    public BodyMove(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        base.UpdateUI();
        float height = 100.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Body Move", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

        if (ImGui.Button("Explode"))
        {
            World.Explode(WorldId, ExplosionPosition, ExplosionRadius, ExplosionMagnitude);
        }

        ImGui.SliderFloat("Magnitude", ref ExplosionMagnitude, -8.0f, 8.0f, "%.1f");

        ImGui.End();
    }
}