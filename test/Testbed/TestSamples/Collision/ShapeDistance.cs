using System;
using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Collision;

[TestInherit]
public class ShapeDistance : Samples.Collision.ShapeDistance
{
    public ShapeDistance(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 310.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Shape Distance", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

        string[] shapeTypes = ["point", "segment", "triangle", "box"];
        int shapeType = (int)TypeA;
        if (ImGui.Combo("shape A", ref shapeType, shapeTypes, shapeTypes.Length))
        {
            TypeA = (TestShapeType)shapeType;
            ProxyA = MakeProxy(TypeA, RadiusA);
        }

        if (ImGui.SliderFloat("radius A", ref RadiusA, 0.0f, 0.5f, "%.2f"))
        {
            ProxyA.Radius = RadiusA;
        }

        shapeType = (int)TypeB;
        if (ImGui.Combo("shape B", ref shapeType, shapeTypes, shapeTypes.Length))
        {
            TypeB = (TestShapeType)shapeType;
            ProxyB = MakeProxy(TypeB, RadiusB);
        }

        if (ImGui.SliderFloat("radius B", ref RadiusB, 0.0f, 0.5f, "%.2f"))
        {
            ProxyB.Radius = RadiusB;
        }

        ImGui.Separator();

        ImGui.SliderFloat("x offset", ref Transform.P.X, -2.0f, 2.0f, "%.2f");
        ImGui.SliderFloat("y offset", ref Transform.P.Y, -2.0f, 2.0f, "%.2f");

        if (ImGui.SliderFloat("angle", ref Angle, -B2Math.Pi, B2Math.Pi, "%.2f"))
        {
            Transform.Q = B2Math.MakeRot(Angle);
        }

        ImGui.Separator();

        ImGui.Checkbox("show indices", ref ShowIndices);
        ImGui.Checkbox("use cache", ref UseCache);

        ImGui.Separator();

        if (ImGui.Checkbox("draw simplex", ref DrawSimplex))
        {
            SimplexIndex = 0;
        }

        if (DrawSimplex)
        {
            ImGui.SliderInt("index", ref SimplexIndex, 0, SimplexCount - 1);
            SimplexIndex = Math.Clamp(SimplexIndex, 0, SimplexCount - 1);
        }

        ImGui.End();
    }
}