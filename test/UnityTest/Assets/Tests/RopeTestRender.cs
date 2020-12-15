using Box2DSharp.Common;
using Box2DSharp.Ropes;
using ImGuiNET;
using Testbed.TestCases;
using UnityEngine;

namespace Box2DSharp.Testbed.Unity.Tests
{
    [TestInherit]
    public class RopeTestRender : RopeTest
    {
        /// <inheritdoc />
        protected override void OnRender()
        {
            ImGui.SetNextWindowPos(new Vector2(10.0f, 100.0f));
            ImGui.SetNextWindowSize(new Vector2(200.0f, 700.0f));
            ImGui.Begin("Tuning", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

            ImGui.Separator();

            ImGui.PushItemWidth(ImGui.GetWindowWidth() * 0.5f);

            const ImGuiComboFlags comboFlags = 0;
            string[] bendModels = {"Spring", "PBD Ang", "XPBD Ang", "PBD Dist", "PBD Height", "PBD Triangle"};
            string[] stretchModels = {"PBD", "XPBD"};

            ImGui.Text("Rope 1");

            var bendModel1 = (int)Tuning1.BendingModel;
            if (ImGui.BeginCombo("Bend Model##1", bendModels[bendModel1], comboFlags))
            {
                for (var i = 0; i < bendModels.Length; ++i)
                {
                    var isSelected = bendModel1 == i;
                    if (ImGui.Selectable(bendModels[i], isSelected))
                    {
                        bendModel1 = i;
                        Tuning1.BendingModel = (BendingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##B1", ref Tuning1.BendDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##B1", ref Tuning1.BendHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##B1", ref Tuning1.BendStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.Checkbox("Isometric##1", ref Tuning1.Isometric);
            ImGui.Checkbox("Fixed Mass##1", ref Tuning1.FixedEffectiveMass);
            ImGui.Checkbox("Warm Start##1", ref Tuning1.WarmStart);

            var stretchModel1 = (int)Tuning1.StretchingModel;
            if (ImGui.BeginCombo("Stretch Model##1", stretchModels[stretchModel1], comboFlags))
            {
                for (var i = 0; i < stretchModels.Length; ++i)
                {
                    var isSelected = stretchModel1 == i;
                    if (ImGui.Selectable(stretchModels[i], isSelected))
                    {
                        stretchModel1 = i;
                        Tuning1.StretchingModel = (StretchingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##S1", ref Tuning1.StretchDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##S1", ref Tuning1.StretchHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##S1", ref Tuning1.StretchStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.SliderInt("Iterations##1", ref Iterations1, 1, 100, "%d");

            ImGui.Separator();

            ImGui.Text("Rope 2");

            var bendModel2 = (int)Tuning2.BendingModel;
            if (ImGui.BeginCombo("Bend Model##2", bendModels[bendModel2], comboFlags))
            {
                for (var i = 0; i < bendModels.Length; ++i)
                {
                    var isSelected = bendModel2 == i;
                    if (ImGui.Selectable(bendModels[i], isSelected))
                    {
                        bendModel2 = i;
                        Tuning2.BendingModel = (BendingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##", ref Tuning2.BendDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##", ref Tuning2.BendHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##", ref Tuning2.BendStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.Checkbox("Isometric##2", ref Tuning2.Isometric);
            ImGui.Checkbox("Fixed Mass##2", ref Tuning2.FixedEffectiveMass);
            ImGui.Checkbox("Warm Start##2", ref Tuning2.WarmStart);

            var stretchModel2 = (int)Tuning2.StretchingModel;
            if (ImGui.BeginCombo("Stretch Model##2", stretchModels[stretchModel2], comboFlags))
            {
                for (var i = 0; i < stretchModels.Length; ++i)
                {
                    var isSelected = stretchModel2 == i;
                    if (ImGui.Selectable(stretchModels[i], isSelected))
                    {
                        stretchModel2 = i;
                        Tuning2.StretchingModel = (StretchingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##S2", ref Tuning2.StretchDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##S2", ref Tuning2.StretchHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##S2", ref Tuning2.StretchStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.SliderInt("Iterations##2", ref Iterations2, 1, 100, "%d");

            ImGui.Separator();

            ImGui.SliderFloat("Speed", ref Speed, 10.0f, 100.0f, "%.0f");

            if (ImGui.Button("Reset"))
            {
                Position1.Set(-5.0f, 15.0f);
                Position2.Set(5.0f, 15.0f);
                Rope1.Reset(Position1);
                Rope2.Reset(Position2);
            }

            ImGui.PopItemWidth();

            ImGui.End();

            Rope1.Draw(Drawer);
            Rope2.Draw(Drawer);

            DrawString("Press comma and period to move left and right");
            base.OnRender();
        }
    }
}