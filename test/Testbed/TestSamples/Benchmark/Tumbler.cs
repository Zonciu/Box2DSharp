using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Benchmark;

[TestInherit]
public class Tumbler(Settings settings) : Samples.Benchmark.Tumbler(settings)
{
    public override void UpdateUI()
    {
        float height = 60.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(200.0f, height));
        ImGui.Begin("Benchmark: Tumbler", ImGuiWindowFlags.NoResize);
        ImGui.PushItemWidth(120.0f);

        if (ImGui.SliderFloat("Speed", ref MotorSpeed, 0.0f, 100.0f, "%.f"))
        {
            RevoluteJointFunc.SetMotorSpeed(JointId, B2Math.Pi / 180.0f * MotorSpeed);

            if (MotorSpeed > 0.0f)
            {
                Joint.WakeBodies(JointId);
            }
        }

        ImGui.PopItemWidth();
        ImGui.End();
        base.UpdateUI();
    }
}