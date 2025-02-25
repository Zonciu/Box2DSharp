using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Collision;

[TestInherit]
public class OverlapWorld : Samples.Collision.OverlapWorld
{
    public OverlapWorld(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        base.UpdateUI();
        float height = 330.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(140.0f, height));

        ImGui.Begin("Overlap World", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

        if (ImGui.Button("Polygon 1"))
            Create(0);
        ImGui.SameLine();
        if (ImGui.Button("10x##Poly1"))
            CreateN(0, 10);

        if (ImGui.Button("Polygon 2"))
            Create(1);
        ImGui.SameLine();
        if (ImGui.Button("10x##Poly2"))
            CreateN(1, 10);

        if (ImGui.Button("Polygon 3"))
            Create(2);
        ImGui.SameLine();
        if (ImGui.Button("10x##Poly3"))
            CreateN(2, 10);

        if (ImGui.Button("Box"))
            Create(3);
        ImGui.SameLine();
        if (ImGui.Button("10x##Box"))
            CreateN(3, 10);

        if (ImGui.Button("Circle"))
            Create(4);
        ImGui.SameLine();
        if (ImGui.Button("10x##Circle"))
            CreateN(4, 10);

        if (ImGui.Button("Capsule"))
            Create(5);
        ImGui.SameLine();
        if (ImGui.Button("10x##Capsule"))
            CreateN(5, 10);

        if (ImGui.Button("Segment"))
            Create(6);
        ImGui.SameLine();
        if (ImGui.Button("10x##Segment"))
            CreateN(6, 10);

        if (ImGui.Button("Destroy Shape"))
        {
            DestroyBody();
        }

        ImGui.Separator();
        ImGui.Text("Overlap Shape");
        ImGui.RadioButton("Circle##Overlap", ref ShapeType, TestShapeType.CircleShape);
        ImGui.RadioButton("Capsule##Overlap", ref ShapeType, TestShapeType.CapsuleShape);
        ImGui.RadioButton("Box##Overlap", ref ShapeType, TestShapeType.BoxShape);

        ImGui.End();
    }
}