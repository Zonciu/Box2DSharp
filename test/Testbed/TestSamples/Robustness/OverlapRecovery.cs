using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Robustness;

[TestInherit]
public class OverlapRecovery : Samples.Robustness.OverlapRecovery
{
    public OverlapRecovery(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 210.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(220.0f, height));

        ImGui.Begin("Overlap Recovery", ImGuiWindowFlags.NoResize);
        ImGui.PushItemWidth(100.0f);

        bool changed = false;
        changed = changed || ImGui.SliderFloat("Extent", ref Extent, 0.1f, 1.0f, "%.1f");
        changed = changed || ImGui.SliderInt("Base Count", ref BaseCount, 1, 10);
        changed = changed || ImGui.SliderFloat("Overlap", ref Overlap, 0.0f, 1.0f, "%.2f");
        changed = changed || ImGui.SliderFloat("Pushout", ref PushOut, 0.0f, 10.0f, "%.1f");
        changed = changed || ImGui.SliderFloat("Hertz", ref Hertz, 0.0f, 120.0f, "%.F");
        changed = changed || ImGui.SliderFloat("Damping Ratio", ref DampingRatio, 0.0f, 20.0f, "%.1f");
        changed = changed || ImGui.Button("Reset Scene");

        if (changed)
        {
            CreateScene();
        }

        ImGui.PopItemWidth();
        ImGui.End();
    }
}

// This sample shows how careful creation of compound shapes leads to better simulation and avoids
// objects getting stuck.
// This also shows how to get the combined AABB for the body.

// Restitution is approximate since Box2D uses speculative collision

// This sample shows how to modify the geometry on an existing shape. This is only supported on
// dynamic and kinematic shapes because static shapes don't look for new collisions.