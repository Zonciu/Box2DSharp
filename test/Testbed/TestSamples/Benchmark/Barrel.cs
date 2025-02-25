using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Benchmark;

[TestInherit]
public class Barrel(Settings settings) : Samples.Benchmark.Barrel(settings)
{
    private static readonly string[] _shapeTypes = ["Circle", "Capsule", "Mix", "Compound", "Human"];

    public override void UpdateUI()
    {
        float height = 80.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(220.0f, height));
        ImGui.Begin("Benchmark: Barrel", ImGuiWindowFlags.NoResize);

        bool changed = false;

        int shapeType = (int)ShapeType;
        changed = changed || ImGui.Combo("Shape", ref shapeType, _shapeTypes, _shapeTypes.Length);
        ShapeType = (TestShapeType)shapeType;

        changed = changed || ImGui.Button("Reset Scene");

        if (changed)
        {
            CreateScene();
        }

        ImGui.End();
        base.UpdateUI();
    }
}