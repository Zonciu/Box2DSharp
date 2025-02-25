using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

public class Cantilever : Samples.Joints.Cantilever
{
    public Cantilever(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 180.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Cantilever", ImGuiWindowFlags.NoResize);
        ImGui.PushItemWidth(100.0f);

        if (ImGui.SliderFloat("Linear Hertz", ref m_linearHertz, 0.0f, 20.0f, "%.0f"))
        {
            for (int i = 0; i < e_count; ++i)
            {
                WeldJointFunc.SetLinearHertz(m_jointIds[i], m_linearHertz);
            }
        }

        if (ImGui.SliderFloat("Linear Damping Ratio", ref m_linearDampingRatio, 0.0f, 10.0f, "%.1f"))
        {
            for (int i = 0; i < e_count; ++i)
            {
                WeldJointFunc.SetLinearDampingRatio(m_jointIds[i], m_linearDampingRatio);
            }
        }

        if (ImGui.SliderFloat("Angular Hertz", ref m_angularHertz, 0.0f, 20.0f, "%.0f"))
        {
            for (int i = 0; i < e_count; ++i)
            {
                WeldJointFunc.SetAngularHertz(m_jointIds[i], m_angularHertz);
            }
        }

        if (ImGui.SliderFloat("Angular Damping Ratio", ref m_angularDampingRatio, 0.0f, 10.0f, "%.1f"))
        {
            for (int i = 0; i < e_count; ++i)
            {
                WeldJointFunc.SetAngularDampingRatio(m_jointIds[i], m_angularDampingRatio);
            }
        }

        if (ImGui.Checkbox("Collide Connected", ref m_collideConnected))
        {
            for (int i = 0; i < e_count; ++i)
            {
                Joint.SetCollideConnected(m_jointIds[i], m_collideConnected);
            }
        }

        if (ImGui.SliderFloat("Gravity Scale", ref m_gravityScale, -1.0f, 1.0f, "%.1f"))
        {
            for (int i = 0; i < e_count; ++i)
            {
                Body.SetGravityScale(m_bodyIds[i], m_gravityScale);
            }
        }

        ImGui.PopItemWidth();
        ImGui.End();
    }
}