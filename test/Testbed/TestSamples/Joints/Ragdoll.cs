using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

[TestInherit]
public class Ragdoll : Samples.Joints.Ragdoll
{
    public Ragdoll(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 140.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(180.0f, height));

        ImGui.Begin("Ragdoll", ImGuiWindowFlags.NoResize);
        ImGui.PushItemWidth(100.0f);

        if (ImGui.SliderFloat("Friction", ref m_jointFrictionTorque, 0.0f, 1.0f, "%3.2f"))
        {
            m_human.SetJointFrictionTorque(m_jointFrictionTorque);
        }

        if (ImGui.SliderFloat("Hertz", ref m_jointHertz, 0.0f, 10.0f, "%3.1f"))
        {
            m_human.SetJointSpringHertz(m_jointHertz);
        }

        if (ImGui.SliderFloat("Damping", ref m_jointDampingRatio, 0.0f, 4.0f, "%3.1f"))
        {
            m_human.SetJointDampingRatio(m_jointDampingRatio);
        }

        ImGui.PopItemWidth();
        ImGui.End();
    }
}