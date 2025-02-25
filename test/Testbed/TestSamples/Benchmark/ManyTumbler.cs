using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Benchmark;

[TestInherit]
public class ManyTumbler(Settings settings) : Samples.Benchmark.ManyTumbler(settings)
{
    public override void UpdateUI()
    {
        float height = 110.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(200.0f, height));
        ImGui.Begin("Benchmark: Many Tumblers", ImGuiWindowFlags.NoResize);
        ImGui.PushItemWidth(100.0f);

        bool changed = false;
        changed = changed || ImGui.SliderInt("Row Count", ref RowCount, 1, 32);
        changed = changed || ImGui.SliderInt("Column Count", ref ColumnCount, 1, 32);

        if (changed)
        {
            CreateScene();
        }

        if (ImGui.SliderFloat("Speed", ref AngularSpeed, 0.0f, 100.0f, "%.f"))
        {
            for (int i = 0; i < TumblerCount; ++i)
            {
                Body.SetAngularVelocity(TumblerIds[i], (B2Math.Pi / 180.0f) * AngularSpeed);
                Body.SetAwake(TumblerIds[i], true);
            }
        }

        ImGui.PopItemWidth();
        ImGui.End();
        base.UpdateUI();
    }
}