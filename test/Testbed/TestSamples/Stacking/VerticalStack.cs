using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Stacking;

[TestInherit]
public class VerticalStack(Settings settings) : Samples.Stacking.VerticalStack(settings)
{
    public override void UpdateUI()
    {
        float height = 230.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Vertical Stack", ImGuiWindowFlags.NoResize);

        ImGui.PushItemWidth(120.0f);

        bool changed = false;
        string[] shapeTypes = ["Circle", "Box"];

        changed = changed || ImGui.Combo("Shape", ref ShapeType, shapeTypes, shapeTypes.Length);

        changed = changed || ImGui.SliderInt("Rows", ref RowCount, 1, e_maxRows);
        changed = changed || ImGui.SliderInt("Columns", ref ColumnCount, 1, e_maxColumns);

        ImGui.SliderInt("Bullets", ref BulletCount, 1, e_maxBullets);

        ImGui.Combo("Bullet Shape", ref BulletType, shapeTypes, shapeTypes.Length);

        ImGui.PopItemWidth();

        if (ImGui.Button("Fire Bullets") || Input.IsKeyDown(KeyCodes.B))
        {
            DestroyBullets();
            FireBullets();
        }

        if (ImGui.Button("Destroy Body"))
        {
            DestroyBody();
        }

        changed = changed || ImGui.Button("Reset Stack");

        if (changed)
        {
            DestroyBullets();
            CreateStacks();
        }

        ImGui.End();
    }
}