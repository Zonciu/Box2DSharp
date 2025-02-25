using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Benchmark;

[TestInherit]
public class ManyPyramids(Settings settings) : Samples.Benchmark.ManyPyramids(settings)
{
    public override void UpdateUI()
    {
        float height = 160.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(200.0f, height));
        ImGui.Begin("Benchmark: Many Pyramids", ImGuiWindowFlags.NoResize);
        ImGui.PushItemWidth(100.0f);

        bool changed = false;
        changed = changed || ImGui.SliderInt("Row Count", ref RowCount, 1, 32);
        changed = changed || ImGui.SliderInt("Column Count", ref ColumnCount, 1, 32);
        changed = changed || ImGui.SliderInt("Base Count", ref BaseCount, 1, 30);

        changed = changed || ImGui.SliderFloat("Round", ref Round, 0.0f, 0.4f, "%.1f");
        changed = changed || ImGui.Button("Reset Scene");

        if (changed)
        {
            CreateScene();
        }

        ImGui.PopItemWidth();
        ImGui.End();
        base.UpdateUI();
    }
}