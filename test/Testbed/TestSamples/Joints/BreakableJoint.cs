using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

[TestInherit]
public class BreakableJoint : Samples.Joints.BreakableJoint
{
    public BreakableJoint(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 100.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Breakable Joint", ImGuiWindowFlags.NoResize);

        ImGui.SliderFloat("break force", ref m_breakForce, 0.0f, 10000.0f, "%.1f");

        Vec2 gravity = World.GetGravity(WorldId);
        if (ImGui.SliderFloat("gravity", ref gravity.Y, -50.0f, 50.0f, "%.1f"))
        {
            World.SetGravity(WorldId, gravity);
        }

        ImGui.End();
    }
};