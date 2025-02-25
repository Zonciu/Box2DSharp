using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Shapes;

[TestInherit]
public class CompoundShapes : Samples.Shapes.CompoundShapes
{
    public CompoundShapes(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 100.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(180.0f, height));

        ImGui.Begin("Compound Shapes", ImGuiWindowFlags.NoResize);

        if (ImGui.Button("Intrude"))
        {
            Spawn();
        }

        ImGui.Checkbox("Body AABBs", ref DrawBodyAABBs);

        ImGui.End();
    }
};