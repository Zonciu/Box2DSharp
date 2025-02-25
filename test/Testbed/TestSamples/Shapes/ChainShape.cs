using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Shapes;

[TestInherit]
public class ChainShape : Samples.Shapes.ChainShape
{
    public ChainShape(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 135.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Chain Shape", ImGuiWindowFlags.NoResize);

        string[] shapeTypes = ["Circle", "Capsule", "Box"];

        if (ImGui.Combo("Shape", ref ShapeType, shapeTypes, shapeTypes.Length))
        {
            Launch();
        }

        if (ImGui.SliderFloat("Friction", ref Friction, 0.0f, 1.0f, "%.2f"))
        {
            Shape.SetFriction(ShapeId, Friction);
            Shape.SetFriction(ChainId, Friction);
        }

        if (ImGui.SliderFloat("Restitution", ref Restitution, 0.0f, 2.0f, "%.1f"))
        {
            Shape.SetRestitution(ShapeId, Restitution);
        }

        if (ImGui.Button("Launch"))
        {
            Launch();
        }

        ImGui.End();
    }
}