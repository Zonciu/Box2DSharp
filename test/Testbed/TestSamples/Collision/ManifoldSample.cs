using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Collision;

[TestInherit]
public class ManifoldSample : Samples.Collision.ManifoldSample
{
    public ManifoldSample(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        base.UpdateUI();
        float height = 300.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Manifold", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

        ImGui.SliderFloat("x offset", ref m_transform.P.X, -2.0f, 2.0f, "%.2f");
        ImGui.SliderFloat("y offset", ref m_transform.P.Y, -2.0f, 2.0f, "%.2f");

        if (ImGui.SliderFloat("angle", ref m_angle, -B2Math.Pi, B2Math.Pi, "%.2f"))
        {
            m_transform.Q = B2Math.MakeRot(m_angle);
        }

        ImGui.SliderFloat("round", ref m_round, 0.0f, 0.4f, "%.1f");
        ImGui.Checkbox("show ids", ref m_showIds);
        ImGui.Checkbox("show separation", ref m_showSeparation);
        ImGui.Checkbox("show anchors", ref m_showAnchors);
        ImGui.Checkbox("enable caching", ref m_enableCaching);

        if (ImGui.Button("Reset"))
        {
            m_transform = Transform.Identity;
            m_angle = 0.0f;
        }

        ImGui.Separator();

        ImGui.Text("mouse button 1: drag");
        ImGui.Text("mouse button 1 + shift: rotate");

        ImGui.End();
    }
}