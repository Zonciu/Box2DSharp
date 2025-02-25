using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

[TestInherit]
public class PrismaticJointSample : Samples.Joints.PrismaticJointSample
{
    public PrismaticJointSample(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 220.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Prismatic Joint", ImGuiWindowFlags.NoResize);

        if (ImGui.Checkbox("Limit", ref m_enableLimit))
        {
            PrismaticJointFunc.EnableLimit(m_jointId, m_enableLimit);
            Joint.WakeBodies(m_jointId);
        }

        if (ImGui.Checkbox("Motor", ref m_enableMotor))
        {
            PrismaticJointFunc.EnableMotor(m_jointId, m_enableMotor);
            Joint.WakeBodies(m_jointId);
        }

        if (m_enableMotor)
        {
            if (ImGui.SliderFloat("Max Force", ref m_motorForce, 0.0f, 200.0f, "%.0f"))
            {
                PrismaticJointFunc.SetMaxMotorForce(m_jointId, m_motorForce);
                Joint.WakeBodies(m_jointId);
            }

            if (ImGui.SliderFloat("Speed", ref m_motorSpeed, -40.0f, 40.0f, "%.0f"))
            {
                PrismaticJointFunc.SetMotorSpeed(m_jointId, m_motorSpeed);
                Joint.WakeBodies(m_jointId);
            }
        }

        if (ImGui.Checkbox("Spring", ref m_enableSpring))
        {
            PrismaticJointFunc.EnableSpring(m_jointId, m_enableSpring);
            Joint.WakeBodies(m_jointId);
        }

        if (m_enableSpring)
        {
            if (ImGui.SliderFloat("Hertz", ref m_hertz, 0.0f, 10.0f, "%.1f"))
            {
                PrismaticJointFunc.SetSpringHertz(m_jointId, m_hertz);
                Joint.WakeBodies(m_jointId);
            }

            if (ImGui.SliderFloat("Damping", ref m_dampingRatio, 0.0f, 2.0f, "%.1f"))
            {
                PrismaticJointFunc.SetSpringDampingRatio(m_jointId, m_dampingRatio);
                Joint.WakeBodies(m_jointId);
            }
        }

        ImGui.End();
    }
}