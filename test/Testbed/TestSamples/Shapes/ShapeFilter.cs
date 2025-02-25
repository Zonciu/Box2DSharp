using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Shapes;

[TestInherit]
public class ShapeFilter : Samples.Shapes.ShapeFilter
{
    public ShapeFilter(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 240.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(240.0f, height));

        ImGui.Begin("Shape Filter", ImGuiWindowFlags.NoResize);

        ImGui.Text("Player 1 Collides With");
        {
            Filter filter1 = Shape.GetFilter(Shape1Id);
            bool team2 = (filter1.MaskBits & CollisionBits.Team2) == CollisionBits.Team2;
            if (ImGui.Checkbox("Team 2##1", ref team2))
            {
                if (team2)
                {
                    filter1.MaskBits |= CollisionBits.Team2;
                }
                else
                {
                    filter1.MaskBits &= ~CollisionBits.Team2;
                }

                Shape.SetFilter(Shape1Id, filter1);
            }

            bool team3 = (filter1.MaskBits & CollisionBits.Team3) == CollisionBits.Team3;
            if (ImGui.Checkbox("Team 3##1", ref team3))
            {
                if (team3)
                {
                    filter1.MaskBits |= CollisionBits.Team3;
                }
                else
                {
                    filter1.MaskBits &= ~CollisionBits.Team3;
                }

                Shape.SetFilter(Shape1Id, filter1);
            }
        }

        ImGui.Separator();

        ImGui.Text("Player 2 Collides With");
        {
            Filter filter2 = Shape.GetFilter(Shape2Id);
            bool team1 = (filter2.MaskBits & CollisionBits.Team1) == CollisionBits.Team1;
            if (ImGui.Checkbox("Team 1##2", ref team1))
            {
                if (team1)
                {
                    filter2.MaskBits |= CollisionBits.Team1;
                }
                else
                {
                    filter2.MaskBits &= ~CollisionBits.Team1;
                }

                Shape.SetFilter(Shape2Id, filter2);
            }

            bool team3 = (filter2.MaskBits & CollisionBits.Team3) == CollisionBits.Team3;
            if (ImGui.Checkbox("Team 3##2", ref team3))
            {
                if (team3)
                {
                    filter2.MaskBits |= CollisionBits.Team3;
                }
                else
                {
                    filter2.MaskBits &= ~CollisionBits.Team3;
                }

                Shape.SetFilter(Shape2Id, filter2);
            }
        }

        ImGui.Separator();

        ImGui.Text("Player 3 Collides With");
        {
            Filter filter3 = Shape.GetFilter(Shape3Id);
            bool team1 = (filter3.MaskBits & CollisionBits.Team1) == CollisionBits.Team1;
            if (ImGui.Checkbox("Team 1##3", ref team1))
            {
                if (team1)
                {
                    filter3.MaskBits |= CollisionBits.Team1;
                }
                else
                {
                    filter3.MaskBits &= ~CollisionBits.Team1;
                }

                Shape.SetFilter(Shape3Id, filter3);
            }

            bool team2 = (filter3.MaskBits & CollisionBits.Team2) == CollisionBits.Team2;
            if (ImGui.Checkbox("Team 2##3", ref team2))
            {
                if (team2)
                {
                    filter3.MaskBits |= CollisionBits.Team2;
                }
                else
                {
                    filter3.MaskBits &= ~CollisionBits.Team2;
                }

                Shape.SetFilter(Shape3Id, filter3);
            }
        }

        ImGui.End();
    }
}