using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Bodies;

[TestInherit]
public class Weeble : Samples.Bodies.Weeble
{
    public Weeble(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 120.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(200.0f, height));
        ImGui.Begin("Weeble", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);
        if (ImGui.Button("Teleport"))
        {
            Body.SetTransform(WeebleId, (0.0f, 5.0f), B2Math.MakeRot(0.95f * B2Math.Pi));
        }

        if (ImGui.Button("Explode"))
        {
            World.Explode(WorldId, ExplosionPosition, ExplosionRadius, ExplosionMagnitude);
        }

        ImGui.PushItemWidth(100.0f);

        ImGui.SliderFloat("Magnitude", ref ExplosionMagnitude, -100.0f, 100.0f, "%.1f");

        ImGui.PopItemWidth();
        ImGui.End();
    }
}