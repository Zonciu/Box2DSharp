using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

/// This test shows how to use a motor joint. A motor joint
/// can be used to animate a dynamic body. With finite motor forces
/// the body can be blocked by collision with other bodies.
///	By setting the correction factor to zero, the motor joint acts
///	like top-down dry friction.
[TestInherit]
public class MotorJointSample : Samples.Joints.MotorJointSample
{
    public MotorJointSample(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 140.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Motor Joint", ImGuiWindowFlags.NoResize);

        if (ImGui.Checkbox("Go", ref m_go))
        { }

        if (ImGui.SliderFloat("Max Force", ref m_maxForce, 0.0f, 1000.0f, "%.0f"))
        {
            MotorJointFunc.SetMaxForce(m_jointId, m_maxForce);
        }

        if (ImGui.SliderFloat("Max Torque", ref m_maxTorque, 0.0f, 1000.0f, "%.0f"))
        {
            MotorJointFunc.SetMaxTorque(m_jointId, m_maxTorque);
        }

        if (ImGui.SliderFloat("Correction", ref m_correctionFactor, 0.0f, 1.0f, "%.1f"))
        {
            MotorJointFunc.SetCorrectionFactor(m_jointId, m_correctionFactor);
        }

        ImGui.End();
    }
}