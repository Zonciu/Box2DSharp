using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

[TestInherit]
public class WheelJointSample : Samples.Joints.WheelJointSample
{
    public WheelJointSample(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 220.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Wheel Joint", ImGuiWindowFlags.NoResize);

        if (ImGui.Checkbox("Limit", ref m_enableLimit))
        {
            WheelJointFunc.EnableLimit(m_jointId, m_enableLimit);
        }

        if (ImGui.Checkbox("Motor", ref m_enableMotor))
        {
            WheelJointFunc.EnableMotor(m_jointId, m_enableMotor);
        }

        if (m_enableMotor)
        {
            if (ImGui.SliderFloat("Torque", ref m_motorTorque, 0.0f, 20.0f, "%.0f"))
            {
                WheelJointFunc.SetMaxMotorTorque(m_jointId, m_motorTorque);
            }

            if (ImGui.SliderFloat("Speed", ref m_motorSpeed, -20.0f, 20.0f, "%.0f"))
            {
                WheelJointFunc.SetMotorSpeed(m_jointId, m_motorSpeed);
            }
        }

        if (ImGui.Checkbox("Spring", ref m_enableSpring))
        {
            WheelJointFunc.EnableSpring(m_jointId, m_enableSpring);
        }

        if (m_enableSpring)
        {
            if (ImGui.SliderFloat("Hertz", ref m_hertz, 0.0f, 10.0f, "%.1f"))
            {
                WheelJointFunc.SetSpringHertz(m_jointId, m_hertz);
            }

            if (ImGui.SliderFloat("Damping", ref m_dampingRatio, 0.0f, 2.0f, "%.1f"))
            {
                WheelJointFunc.SetSpringDampingRatio(m_jointId, m_dampingRatio);
            }
        }

        ImGui.End();
    }
}