using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

[TestInherit]
public class Bridge : Samples.Joints.Bridge
{
    public Bridge(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 80.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Bridge", ImGuiWindowFlags.NoResize);

        // Slider takes half the window
        ImGui.PushItemWidth(ImGui.GetWindowWidth() * 0.5f);
        bool updateFriction = ImGui.SliderFloat("Joint Friction", ref m_frictionTorque, 0.0f, 1000.0f, "%2.F");
        if (updateFriction)
        {
            for (int i = 0; i <= e_count; ++i)
            {
                RevoluteJointFunc.SetMaxMotorTorque(m_jointIds[i], m_frictionTorque);
            }
        }

        if (ImGui.SliderFloat("Gravity scale", ref m_gravityScale, -1.0f, 1.0f, "%.1f"))
        {
            for (int i = 0; i < e_count; ++i)
            {
                Body.SetGravityScale(m_bodyIds[i], m_gravityScale);
            }
        }

        ImGui.End();
    }
}