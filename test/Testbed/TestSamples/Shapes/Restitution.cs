using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Shapes;

[TestInherit]
public class Restitution(Settings settings) : Samples.Shapes.Restitution(settings)
{
    public override void UpdateUI()
    {
        float height = 100.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Restitution", ImGuiWindowFlags.NoResize);

        bool changed = false;
        string[] shapeTypes = ["Circle", "Box"];

        changed = changed || ImGui.Combo("Shape", ref ShapeType, shapeTypes, shapeTypes.Length);

        changed = changed || ImGui.Button("Reset");

        if (changed)
        {
            CreateBodies();
        }

        ImGui.End();
    }
}