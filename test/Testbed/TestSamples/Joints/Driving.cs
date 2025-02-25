using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

[TestInherit]
public class Driving : Samples.Joints.Driving
{
    public Driving(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 140.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(200.0f, height));

        ImGui.Begin("Driving", ImGuiWindowFlags.NoResize);

        ImGui.PushItemWidth(100.0f);
        if (ImGui.SliderFloat("Spring Hertz", ref m_hertz, 0.0f, 20.0f, "%.0f"))
        {
            m_car.SetHertz(m_hertz);
        }

        if (ImGui.SliderFloat("Damping Ratio", ref m_dampingRatio, 0.0f, 10.0f, "%.1f"))
        {
            m_car.SetDampingRadio(m_dampingRatio);
        }

        if (ImGui.SliderFloat("Speed", ref m_speed, 0.0f, 50.0f, "%.0f"))
        {
            m_car.SetSpeed(m_throttle * m_speed);
        }

        if (ImGui.SliderFloat("Torque", ref m_torque, 0.0f, 5.0f, "%.1f"))
        {
            m_car.SetTorque(m_torque);
        }

        ImGui.PopItemWidth();

        ImGui.End();
    }
}