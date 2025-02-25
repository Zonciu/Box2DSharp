using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Collision;

[TestInherit]
public class SmoothManifold : Samples.Collision.SmoothManifold
{
    public SmoothManifold(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        base.UpdateUI();

        float height = 290.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(180.0f, height));

        ImGui.Begin("Smooth Manifold", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);
        ImGui.PushItemWidth(100.0f);

        {
            string[] shapeTypes = ["Circle", "Box"];
            ImGui.Combo("Shape", ref m_shapeType, shapeTypes, shapeTypes.Length);
        }

        ImGui.SliderFloat("x Offset", ref m_transform.P.X, -2.0f, 2.0f, "%.2f");
        ImGui.SliderFloat("y Offset", ref m_transform.P.Y, -2.0f, 2.0f, "%.2f");

        if (ImGui.SliderFloat("Angle", ref m_angle, -B2Math.Pi, B2Math.Pi, "%.2f"))
        {
            m_transform.Q = B2Math.MakeRot(m_angle);
        }

        ImGui.SliderFloat("Round", ref m_round, 0.0f, 0.4f, "%.1f");
        ImGui.Checkbox("Show Ids", ref m_showIds);
        ImGui.Checkbox("Show Separation", ref m_showSeparation);
        ImGui.Checkbox("Show Anchors", ref m_showAnchors);

        if (ImGui.Button("Reset"))
        {
            m_transform = Transform.Identity;
            m_angle = 0.0f;
        }

        ImGui.Separator();

        ImGui.Text("mouse button 1: drag");
        ImGui.Text("mouse button 1 + shift: rotate");

        ImGui.PopItemWidth();
        ImGui.End();
    }
}