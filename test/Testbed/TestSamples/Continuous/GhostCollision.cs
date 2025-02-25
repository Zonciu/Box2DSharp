using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Continuous;

[TestInherit]
public class GhostCollision : Samples.Continuous.GhostCollision
{
    public GhostCollision(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        base.UpdateUI();
        float height = 160.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(180.0f, height));

        ImGui.Begin("Ghost Collision", ImGuiWindowFlags.NoResize);
        ImGui.PushItemWidth(100.0f);

        if (ImGui.Checkbox("Chain", ref m_useChain))
        {
            CreateScene();
        }

        if (m_useChain == false)
        {
            if (ImGui.SliderFloat("Bevel", ref m_bevel, 0.0f, 1.0f, "%.2f"))
            {
                CreateScene();
            }
        }

        {
            string[] shapeTypes = ["Circle", "Capsule", "Box"];
            ImGui.Combo("Shape", ref m_shapeType, shapeTypes, shapeTypes.Length);
        }

        if (m_shapeType == TestShapeType.e_boxShape)
        {
            ImGui.SliderFloat("Round", ref m_round, 0.0f, 0.4f, "%.1f");
        }

        if (ImGui.SliderFloat("Friction", ref m_friction, 0.0f, 1.0f, "%.1f"))
        {
            if (m_shapeId.IsNotNull)
            {
                Shape.SetFriction(m_shapeId, m_friction);
            }

            CreateScene();
        }

        if (ImGui.Button("Launch"))
        {
            Launch();
        }

        ImGui.PopItemWidth();
        ImGui.End();
    }
}