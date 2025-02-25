using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Benchmark;

[TestInherit]
public class JointGrid(Settings settings) : Samples.Benchmark.JointGrid(settings)
{
    public override void UpdateUI()
    {
        float height = 60.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));
        ImGui.Begin("Benchmark: Joint Grid", ImGuiWindowFlags.NoResize);

        if (ImGui.SliderFloat("gravity", ref Gravity, 0.0f, 20.0f, "%.1f"))
        {
            World.SetGravity(WorldId, (0.0f, -Gravity));
        }

        ImGui.End();

        base.UpdateUI();
    }
}