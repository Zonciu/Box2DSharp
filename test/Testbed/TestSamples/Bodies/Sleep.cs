using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Bodies;

[TestInherit]
public class Sleep : Samples.Bodies.Sleep
{
    public Sleep(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 100.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));
        ImGui.Begin("Sleep", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

        ImGui.PushItemWidth(120.0f);

        ImGui.Text("Pendulum Tuning");

        float sleepVelocity = Body.GetSleepThreshold(PendulumId);
        if (ImGui.SliderFloat("sleep velocity", ref sleepVelocity, 0.0f, 1.0f, "%.2f"))
        {
            Body.SetSleepThreshold(PendulumId, sleepVelocity);
            Body.SetAwake(PendulumId, true);
        }

        float angularDamping = Body.GetAngularDamping(PendulumId);
        if (ImGui.SliderFloat("angular damping", ref angularDamping, 0.0f, 2.0f, "%.2f"))
        {
            Body.SetAngularDamping(PendulumId, angularDamping);
        }

        ImGui.PopItemWidth();

        ImGui.End();
    }
}