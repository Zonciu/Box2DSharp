using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

[TestInherit]
public class BallAndChain : Samples.Joints.BallAndChain
{
    public BallAndChain(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 60.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Ball and Chain", ImGuiWindowFlags.NoResize);

        bool updateFriction = ImGui.SliderFloat("Joint Friction", ref m_frictionTorque, 0.0f, 1000.0f, "%2.F");
        if (updateFriction)
        {
            for (int i = 0; i <= e_count; ++i)
            {
                RevoluteJointFunc.SetMaxMotorTorque(m_jointIds[i], m_frictionTorque);
            }
        }

        ImGui.End();
    }
}