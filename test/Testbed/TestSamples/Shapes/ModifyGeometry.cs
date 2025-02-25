using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Shapes;

[TestInherit]
public class ModifyGeometry(Settings settings) : Samples.Shapes.ModifyGeometry(settings)
{
    public override void UpdateUI()
    {
        float height = 230.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(200.0f, height));

        ImGui.Begin("Modify Geometry", ImGuiWindowFlags.NoResize);

        if (ImGui.RadioButton("Circle", ShapeType == ShapeType.CircleShape))
        {
            ShapeType = ShapeType.CircleShape;
            UpdateShape();
        }

        if (ImGui.RadioButton("Capsule", ShapeType == ShapeType.CapsuleShape))
        {
            ShapeType = ShapeType.CapsuleShape;
            UpdateShape();
        }

        if (ImGui.RadioButton("Segment", ShapeType == ShapeType.SegmentShape))
        {
            ShapeType = ShapeType.SegmentShape;
            UpdateShape();
        }

        if (ImGui.RadioButton("Polygon", ShapeType == ShapeType.PolygonShape))
        {
            ShapeType = ShapeType.PolygonShape;
            UpdateShape();
        }

        if (ImGui.SliderFloat("Scale", ref Scale, 0.1f, 10.0f, "%.2f"))
        {
            UpdateShape();
        }

        BodyId bodyId = Shape.GetBody(ShapeId);
        BodyType bodyType = Body.GetType(bodyId);

        if (ImGui.RadioButton("Static", bodyType == BodyType.StaticBody))
        {
            Body.SetType(bodyId, BodyType.StaticBody);
        }

        if (ImGui.RadioButton("Kinematic", bodyType == BodyType.KinematicBody))
        {
            Body.SetType(bodyId, BodyType.KinematicBody);
        }

        if (ImGui.RadioButton("Dynamic", bodyType == BodyType.DynamicBody))
        {
            Body.SetType(bodyId, BodyType.DynamicBody);
        }

        ImGui.End();
    }
}