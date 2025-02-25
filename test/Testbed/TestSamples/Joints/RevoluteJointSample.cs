using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

[TestInherit]
public class RevoluteJointSample : Samples.Joints.RevoluteJointSample
{
    public RevoluteJointSample(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 220.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Revolute Joint", ImGuiWindowFlags.NoResize);

        if (ImGui.Checkbox("Limit", ref m_enableLimit))
        {
            RevoluteJointFunc.EnableLimit(m_jointId1, m_enableLimit);
            Joint.WakeBodies(m_jointId1);
        }

        if (ImGui.Checkbox("Motor", ref m_enableMotor))
        {
            RevoluteJointFunc.EnableMotor(m_jointId1, m_enableMotor);
            Joint.WakeBodies(m_jointId1);
        }

        if (m_enableMotor)
        {
            if (ImGui.SliderFloat("Max Torque", ref m_motorTorque, 0.0f, 5000.0f, "%.0f"))
            {
                RevoluteJointFunc.SetMaxMotorTorque(m_jointId1, m_motorTorque);
                Joint.WakeBodies(m_jointId1);
            }

            if (ImGui.SliderFloat("Speed", ref m_motorSpeed, -20.0f, 20.0f, "%.0f"))
            {
                RevoluteJointFunc.SetMotorSpeed(m_jointId1, m_motorSpeed);
                Joint.WakeBodies(m_jointId1);
            }
        }

        if (ImGui.Checkbox("Spring", ref m_enableSpring))
        {
            RevoluteJointFunc.EnableSpring(m_jointId1, m_enableSpring);
            Joint.WakeBodies(m_jointId1);
        }

        if (m_enableSpring)
        {
            if (ImGui.SliderFloat("Hertz", ref m_hertz, 0.0f, 10.0f, "%.1f"))
            {
                RevoluteJointFunc.SetSpringHertz(m_jointId1, m_hertz);
                Joint.WakeBodies(m_jointId1);
            }

            if (ImGui.SliderFloat("Damping", ref m_dampingRatio, 0.0f, 2.0f, "%.1f"))
            {
                RevoluteJointFunc.SetSpringDampingRatio(m_jointId1, m_dampingRatio);
                Joint.WakeBodies(m_jointId1);
            }
        }

        ImGui.End();
    }
}