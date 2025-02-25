using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Collision;

[TestInherit]
public class RayCast : Samples.Collision.RayCast
{
    public RayCast(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 230.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(200.0f, height));

        ImGui.Begin("Ray-cast", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

        ImGui.PushItemWidth(100.0f);

        ImGui.SliderFloat("x offset", ref Transform.P.X, -2.0f, 2.0f, "%.2f");
        ImGui.SliderFloat("y offset", ref Transform.P.Y, -2.0f, 2.0f, "%.2f");

        if (ImGui.SliderFloat("angle", ref Angle, -B2Math.Pi, B2Math.Pi, "%.2f"))
        {
            Transform.Q = B2Math.MakeRot(Angle);
        }

        ImGui.Checkbox("show fraction", ref ShowFraction);

        if (ImGui.Button("Reset"))
        {
            Transform = Transform.Identity;
            Angle = 0.0f;
        }

        ImGui.Separator();

        ImGui.Text("mouse btn 1: ray cast");
        ImGui.Text("mouse btn 1 + shift: translate");
        ImGui.Text("mouse btn 1 + ctrl: rotate");

        ImGui.PopItemWidth();

        ImGui.End();
    }
}