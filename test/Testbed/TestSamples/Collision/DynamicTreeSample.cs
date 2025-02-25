using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Collision;

public class DynamicTreeSample : Samples.Collision.DynamicTreeSample
{
    public DynamicTreeSample(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 320.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(200.0f, height));

        ImGui.Begin("Dynamic Tree", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

        ImGui.PushItemWidth(100.0f);

        bool changed = false;

        if (ImGui.SliderInt("rows", ref RowCount, 0, 1000, "%d"))
        {
            changed = true;
        }

        if (ImGui.SliderInt("columns", ref ColumnCount, 0, 1000, "%d"))
        {
            changed = true;
        }

        if (ImGui.SliderFloat("fill", ref Fill, 0.0f, 1.0f, "%.2f"))
        {
            changed = true;
        }

        if (ImGui.SliderFloat("grid", ref Grid, 0.5f, 2.0f, "%.2f"))
        {
            changed = true;
        }

        if (ImGui.SliderFloat("ratio", ref Ratio, 1.0f, 10.0f, "%.2f"))
        {
            changed = true;
        }

        if (ImGui.SliderFloat("move", ref MoveFraction, 0.0f, 1.0f, "%.2f"))
        { }

        if (ImGui.SliderFloat("delta", ref MoveDelta, 0.0f, 1.0f, "%.2f"))
        { }

        if (ImGui.RadioButton("Incremental", UpdateType == UpdateTypes.Incremental))
        {
            UpdateType = UpdateTypes.Incremental;
            changed = true;
        }

        if (ImGui.RadioButton("Full Rebuild", UpdateType == UpdateTypes.FullRebuild))
        {
            UpdateType = UpdateTypes.FullRebuild;
            changed = true;
        }

        if (ImGui.RadioButton("Partial Rebuild", UpdateType == UpdateTypes.PartialRebuild))
        {
            UpdateType = UpdateTypes.PartialRebuild;
            changed = true;
        }

        ImGui.Separator();

        ImGui.Text("mouse button 1: ray cast");
        ImGui.Text("mouse button 1 + shift: query");

        ImGui.PopItemWidth();
        ImGui.End();

        if (changed)
        {
            BuildTree();
        }
    }
}