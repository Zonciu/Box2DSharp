using System;
using System.Numerics;
using Box2DSharp.Common;
using Box2DSharp.Rope;
using ImGuiNET;
using OpenToolkit.Windowing.Common;
using OpenToolkit.Windowing.Common.Input;
using Testbed.Basics;

namespace Testbed.Tests
{
    [TestCase("Rope", "Bending")]
    public class Rope : Test
    {
        public Rope()
        {
            const int N = 20;
            const float L = 0.5f;
            var vertices = new Vector2[N];
            var masses = new float[N];

            for (int i = 0; i < N; ++i)
            {
                vertices[i].Set(0.0f, L * (N - i));
                masses[i] = 1.0f;
            }

            masses[0] = 0.0f;
            masses[1] = 0.0f;
            _tuning1 = new RopeTuning
            {
                bendHertz = 30.0f,
                bendDamping = 4.0f,
                bendStiffness = 1.0f,
                bendingModel = BendingModel.XpdAngleBendingModel,
                isometric = true,
                stretchHertz = 30.0f,
                stretchDamping = 4.0f,
                stretchStiffness = 1.0f,
                stretchingModel = StretchingModel.XpbdStretchingModel
            };

            _tuning2 = new RopeTuning
            {
                bendHertz = 30.0f,
                bendDamping = 0.7f,
                bendStiffness = 1.0f,
                bendingModel = BendingModel.PbdHeightBendingModel,
                isometric = true,
                stretchHertz = 30.0f,
                stretchDamping = 1.0f,
                stretchStiffness = 1.0f,
                stretchingModel = StretchingModel.PbdStretchingModel
            };

            m_position1.Set(-5.0f, 15.0f);
            m_position2.Set(5.0f, 15.0f);

            var def = new RopeDef
            {
                vertices = vertices,
                count = N,
                gravity = new Vector2(0.0f, -10.0f),
                masses = masses,
                position = m_position1,
                tuning = _tuning1
            };
            m_rope1 = new Box2DSharp.Rope.Rope();
            m_rope1.Create(def);

            def.position = m_position2;
            def.tuning = _tuning2;
            m_rope2 = new Box2DSharp.Rope.Rope();
            m_rope2.Create(def);

            m_iterations1 = 8;
            m_iterations2 = 8;

            m_speed = 10.0f;
        }

        /// <inheritdoc />
        protected override void OnRender()
        {
            ImGui.SetNextWindowPos(new Vector2(10.0f, 100.0f));
            ImGui.SetNextWindowSize(new Vector2(200.0f, 700.0f));
            ImGui.Begin("Tuning", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

            ImGui.Separator();

            ImGui.PushItemWidth(ImGui.GetWindowWidth() * 0.5f);

            const ImGuiComboFlags comboFlags = 0;
            string[] bendModels = {"Spring", "PBD Ang", "XPBD Ang", "PBD Dist", "PBD Height"};
            string[] stretchModels = {"PBD", "XPBD"};

            ImGui.Text("Rope 1");

            var bendModel1 = (int)_tuning1.bendingModel;
            if (ImGui.BeginCombo("Bend Model##1", bendModels[bendModel1], comboFlags))
            {
                for (int i = 0; i < bendModels.Length; ++i)
                {
                    bool isSelected = bendModel1 == i;
                    if (ImGui.Selectable(bendModels[i], isSelected))
                    {
                        bendModel1 = i;
                        _tuning1.bendingModel = (BendingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##B1", ref _tuning1.bendDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##B1", ref _tuning1.bendHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##B1", ref _tuning1.bendStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.Checkbox("Isometric##1", ref _tuning1.isometric);
            ImGui.Checkbox("Fixed Mass##1", ref _tuning1.fixedEffectiveMass);
            ImGui.Checkbox("Warm Start##1", ref _tuning1.warmStart);

            var stretchModel1 = (int)_tuning1.stretchingModel;
            if (ImGui.BeginCombo("Stretch Model##1", stretchModels[stretchModel1], comboFlags))
            {
                for (int i = 0; i < stretchModels.Length; ++i)
                {
                    bool isSelected = stretchModel1 == i;
                    if (ImGui.Selectable(stretchModels[i], isSelected))
                    {
                        stretchModel1 = i;
                        _tuning1.stretchingModel = (StretchingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##S1", ref _tuning1.stretchDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##S1", ref _tuning1.stretchHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##S1", ref _tuning1.stretchStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.SliderInt("Iterations##1", ref m_iterations1, 1, 100, "%d");

            ImGui.Separator();

            ImGui.Text("Rope 2");

            var bendModel2 = (int)_tuning2.bendingModel;
            if (ImGui.BeginCombo("Bend Model##2", bendModels[bendModel2], comboFlags))
            {
                for (int i = 0; i < bendModels.Length; ++i)
                {
                    bool isSelected = bendModel2 == i;
                    if (ImGui.Selectable(bendModels[i], isSelected))
                    {
                        bendModel2 = i;
                        _tuning2.bendingModel = (BendingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##", ref _tuning2.bendDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##", ref _tuning2.bendHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##", ref _tuning2.bendStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.Checkbox("Isometric##2", ref _tuning2.isometric);
            ImGui.Checkbox("Fixed Mass##2", ref _tuning2.fixedEffectiveMass);
            ImGui.Checkbox("Warm Start##2", ref _tuning2.warmStart);

            var stretchModel2 = (int)_tuning2.stretchingModel;
            if (ImGui.BeginCombo("Stretch Model##2", stretchModels[stretchModel2], comboFlags))
            {
                for (int i = 0; i < stretchModels.Length; ++i)
                {
                    bool isSelected = stretchModel2 == i;
                    if (ImGui.Selectable(stretchModels[i], isSelected))
                    {
                        stretchModel2 = i;
                        _tuning2.stretchingModel = (StretchingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##S2", ref _tuning2.stretchDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##S2", ref _tuning2.stretchHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##S2", ref _tuning2.stretchStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.SliderInt("Iterations##2", ref m_iterations2, 1, 100, "%d");

            ImGui.Separator();

            ImGui.SliderFloat("Speed", ref m_speed, 10.0f, 100.0f, "%.0f");

            if (ImGui.Button("Reset"))
            {
                m_position1.Set(-5.0f, 15.0f);
                m_position2.Set(5.0f, 15.0f);
                m_rope1.Reset(m_position1);
                m_rope2.Reset(m_position2);
            }

            ImGui.PopItemWidth();

            ImGui.End();

            m_rope1.Draw(Drawer);
            m_rope2.Draw(Drawer);

            DrawString("Press comma and period to move left and right");
        }

        protected override void PreStep()
        {
            var dt = TestSettings.Hertz > 0.0f ? 1.0f / TestSettings.Hertz : 0.0f;
            if (Game.IsKeyDown(Key.Comma))
            {
                m_position1.X -= m_speed * dt;
                m_position2.X -= m_speed * dt;
            }

            if (Game.IsKeyDown(Key.Period))
            {
                m_position1.X += m_speed * dt;
                m_position2.X += m_speed * dt;
            }

            if (TestSettings.Pause && !TestSettings.SingleStep)
            {
                dt = 0.0f;
            }

            m_rope1.SetTuning(_tuning1);
            m_rope2.SetTuning(_tuning2);
            m_rope1.Step(dt, m_iterations1, m_position1);
            m_rope2.Step(dt, m_iterations2, m_position2);
        }

        Box2DSharp.Rope.Rope m_rope1;

        Box2DSharp.Rope.Rope m_rope2;

        RopeTuning _tuning1;

        RopeTuning _tuning2;

        int m_iterations1;

        int m_iterations2;

        Vector2 m_position1;

        Vector2 m_position2;

        float m_speed;
    };
}