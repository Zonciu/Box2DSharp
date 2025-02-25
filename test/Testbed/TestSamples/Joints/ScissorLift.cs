using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

[TestInherit]
public class ScissorLift : Samples.Joints.ScissorLift
{
    public ScissorLift(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 140.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Scissor Lift", ImGuiWindowFlags.NoResize);

        if (ImGui.Checkbox("Motor", ref m_enableMotor))
        {
            DistanceJointFunc.EnableMotor(m_liftJointId, m_enableMotor);
            Joint.WakeBodies(m_liftJointId);
        }

        if (ImGui.SliderFloat("Max Force", ref m_motorForce, 0.0f, 3000.0f, "%.0f"))
        {
            DistanceJointFunc.SetMaxMotorForce(m_liftJointId, m_motorForce);
            Joint.WakeBodies(m_liftJointId);
        }

        if (ImGui.SliderFloat("Speed", ref m_motorSpeed, -0.3f, 0.3f, "%.2f"))
        {
            DistanceJointFunc.SetMotorSpeed(m_liftJointId, m_motorSpeed);
            Joint.WakeBodies(m_liftJointId);
        }

        ImGui.End();
    }
}