using System;
using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.WorldSample;

[TestInherit]
public class LargeWorld(Settings settings) : Samples.WorldSample.LargeWorld(settings)
{
    public override void UpdateUI()
    {
        float height = 160.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Large World", ImGuiWindowFlags.NoResize);

        ImGui.SliderFloat("speed", ref Speed, -400.0f, 400.0f, "%.0f");
        if (ImGui.Button("stop"))
        {
            Speed = 0.0f;
        }

        ImGui.Checkbox("explode", ref Explode);
        ImGui.Checkbox("follow car", ref FollowCar);

        ImGui.Text($"world size = {GridSize * GridCount / 1000.0f} kilometers");
        ImGui.End();
    }

    protected override void OnRender()
    {
        float span = 0.5f * (Period * CycleCount);
        float timeStep = Settings.Hertz > 0.0f ? 1.0f / Settings.Hertz : 0.0f;

        if (Settings.Pause)
        {
            timeStep = 0.0f;
        }

        ViewPosition.X += timeStep * Speed;
        ViewPosition.X = Math.Clamp(ViewPosition.X, -span, span);

        if (Speed != 0.0f)
        {
            Global.Camera.Center = ViewPosition;
        }

        if (FollowCar)
        {
            Global.Camera.Center.X = Body.GetPosition(Car.ChassisId).X;
        }

        float radius = 2.0f;
        if ((StepCount & 0x1) == 0x1 && Explode)
        {
            ExplosionPosition.X = (0.5f + CycleIndex) * Period - span;
            World.Explode(WorldId, ExplosionPosition, radius, 1.0f);
            CycleIndex = (CycleIndex + 1) % CycleCount;
        }

        if (Explode)
        {
            Draw.DrawCircle(ExplosionPosition, radius, B2HexColor.Azure);
        }
    }

    public override void OnKeyDown(KeyInputEventArgs keyInput)
    {
        switch (keyInput.Key)
        {
        case KeyCodes.A:
            Car.SetSpeed(20.0f);
            break;
        case KeyCodes.S:
            Car.SetSpeed(0.0f);
            break;
        case KeyCodes.D:
            Car.SetSpeed(-5.0f);
            break;
        }
    }
}