using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Bodies;

[TestInherit]
public class BodyTypeSample(Settings settings) : Samples.Bodies.BodyTypeSample(settings)
{
    public override void UpdateUI()
    {
        float height = 140.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(180.0f, height));
        ImGui.Begin("Body Type", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

        if (ImGui.RadioButton("Static", Type == BodyType.StaticBody))
        {
            Type = BodyType.StaticBody;
            Body.SetType(PlatformId, BodyType.StaticBody);
            Body.SetType(SecondAttachmentId, BodyType.StaticBody);
            Body.SetType(SecondPayloadId, BodyType.StaticBody);
            Body.SetType(TouchingBodyId, BodyType.StaticBody);
            Body.SetType(FloatingBodyId, BodyType.StaticBody);
        }

        if (ImGui.RadioButton("Kinematic", Type == BodyType.KinematicBody))
        {
            Type = BodyType.KinematicBody;
            Body.SetType(PlatformId, BodyType.KinematicBody);
            Body.SetLinearVelocity(PlatformId, (-Speed, 0.0f));
            Body.SetAngularVelocity(PlatformId, 0.0f);
            Body.SetType(SecondAttachmentId, BodyType.KinematicBody);
            Body.SetType(SecondPayloadId, BodyType.KinematicBody);
            Body.SetType(TouchingBodyId, BodyType.KinematicBody);
            Body.SetType(FloatingBodyId, BodyType.KinematicBody);
        }

        if (ImGui.RadioButton("Dynamic", Type == BodyType.DynamicBody))
        {
            Type = BodyType.DynamicBody;
            Body.SetType(PlatformId, BodyType.DynamicBody);
            Body.SetType(SecondAttachmentId, BodyType.DynamicBody);
            Body.SetType(SecondPayloadId, BodyType.DynamicBody);
            Body.SetType(TouchingBodyId, BodyType.DynamicBody);
            Body.SetType(FloatingBodyId, BodyType.DynamicBody);
        }

        if (ImGui.Checkbox("Enable", ref IsEnabled))
        {
            if (IsEnabled)
            {
                Body.Enable(PlatformId);
                Body.Enable(SecondAttachmentId);
                Body.Enable(SecondPayloadId);
                Body.Enable(TouchingBodyId);
                Body.Enable(FloatingBodyId);

                if (Type == BodyType.KinematicBody)
                {
                    Body.SetLinearVelocity(PlatformId, (-Speed, 0.0f));
                    Body.SetAngularVelocity(PlatformId, 0.0f);
                }
            }
            else
            {
                Body.Disable(PlatformId);
                Body.Disable(SecondAttachmentId);
                Body.Disable(SecondPayloadId);
                Body.Disable(TouchingBodyId);
                Body.Disable(FloatingBodyId);
            }
        }

        ImGui.End();
    }
}