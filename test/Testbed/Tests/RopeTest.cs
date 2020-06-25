using System.Numerics;
using Box2DSharp.Common;
using Box2DSharp.Ropes;
using ImGuiNET;
using OpenToolkit.Windowing.Common.Input;
using Testbed.Basics;

namespace Testbed.Tests
{
    [TestCase("Rope", "Bending")]
    public class RopeTest : Test
    {
        private readonly Rope _rope1;

        private readonly Rope _rope2;

        private readonly RopeTuning _tuning1;

        private readonly RopeTuning _tuning2;

        private int _iterations1;

        private int _iterations2;

        private Vector2 _position1;

        private Vector2 _position2;

        private float _speed;

        public RopeTest()
        {
            const int N = 20;
            const float L = 0.5f;
            var vertices = new Vector2[N];
            var masses = new float[N];

            for (var i = 0; i < N; ++i)
            {
                vertices[i].Set(0.0f, L * (N - i));
                masses[i] = 1.0f;
            }

            masses[0] = 0.0f;
            masses[1] = 0.0f;
            _tuning1 = new RopeTuning
            {
                BendHertz = 30.0f,
                BendDamping = 4.0f,
                BendStiffness = 1.0f,
                BendingModel = BendingModel.XpdAngleBendingModel,
                Isometric = true,
                StretchHertz = 30.0f,
                StretchDamping = 4.0f,
                StretchStiffness = 1.0f,
                StretchingModel = StretchingModel.XpbdStretchingModel
            };

            _tuning2 = new RopeTuning
            {
                BendHertz = 30.0f,
                BendDamping = 0.7f,
                BendStiffness = 1.0f,
                BendingModel = BendingModel.PbdHeightBendingModel,
                Isometric = true,
                StretchHertz = 30.0f,
                StretchDamping = 1.0f,
                StretchStiffness = 1.0f,
                StretchingModel = StretchingModel.PbdStretchingModel
            };

            _position1.Set(-5.0f, 15.0f);
            _position2.Set(5.0f, 15.0f);

            var def = new RopeDef
            {
                Vertices = vertices,
                Count = N,
                Gravity = new Vector2(0.0f, -10.0f),
                Masses = masses,
                Position = _position1,
                Tuning = _tuning1
            };
            _rope1 = new Rope();
            _rope1.Create(def);

            def.Position = _position2;
            def.Tuning = _tuning2;
            _rope2 = new Rope();
            _rope2.Create(def);

            _iterations1 = 8;
            _iterations2 = 8;

            _speed = 10.0f;
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

            var bendModel1 = (int)_tuning1.BendingModel;
            if (ImGui.BeginCombo("Bend Model##1", bendModels[bendModel1], comboFlags))
            {
                for (var i = 0; i < bendModels.Length; ++i)
                {
                    var isSelected = bendModel1 == i;
                    if (ImGui.Selectable(bendModels[i], isSelected))
                    {
                        bendModel1 = i;
                        _tuning1.BendingModel = (BendingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##B1", ref _tuning1.BendDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##B1", ref _tuning1.BendHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##B1", ref _tuning1.BendStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.Checkbox("Isometric##1", ref _tuning1.Isometric);
            ImGui.Checkbox("Fixed Mass##1", ref _tuning1.FixedEffectiveMass);
            ImGui.Checkbox("Warm Start##1", ref _tuning1.WarmStart);

            var stretchModel1 = (int)_tuning1.StretchingModel;
            if (ImGui.BeginCombo("Stretch Model##1", stretchModels[stretchModel1], comboFlags))
            {
                for (var i = 0; i < stretchModels.Length; ++i)
                {
                    var isSelected = stretchModel1 == i;
                    if (ImGui.Selectable(stretchModels[i], isSelected))
                    {
                        stretchModel1 = i;
                        _tuning1.StretchingModel = (StretchingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##S1", ref _tuning1.StretchDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##S1", ref _tuning1.StretchHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##S1", ref _tuning1.StretchStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.SliderInt("Iterations##1", ref _iterations1, 1, 100, "%d");

            ImGui.Separator();

            ImGui.Text("Rope 2");

            var bendModel2 = (int)_tuning2.BendingModel;
            if (ImGui.BeginCombo("Bend Model##2", bendModels[bendModel2], comboFlags))
            {
                for (var i = 0; i < bendModels.Length; ++i)
                {
                    var isSelected = bendModel2 == i;
                    if (ImGui.Selectable(bendModels[i], isSelected))
                    {
                        bendModel2 = i;
                        _tuning2.BendingModel = (BendingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##", ref _tuning2.BendDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##", ref _tuning2.BendHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##", ref _tuning2.BendStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.Checkbox("Isometric##2", ref _tuning2.Isometric);
            ImGui.Checkbox("Fixed Mass##2", ref _tuning2.FixedEffectiveMass);
            ImGui.Checkbox("Warm Start##2", ref _tuning2.WarmStart);

            var stretchModel2 = (int)_tuning2.StretchingModel;
            if (ImGui.BeginCombo("Stretch Model##2", stretchModels[stretchModel2], comboFlags))
            {
                for (var i = 0; i < stretchModels.Length; ++i)
                {
                    var isSelected = stretchModel2 == i;
                    if (ImGui.Selectable(stretchModels[i], isSelected))
                    {
                        stretchModel2 = i;
                        _tuning2.StretchingModel = (StretchingModel)i;
                    }

                    if (isSelected)
                    {
                        ImGui.SetItemDefaultFocus();
                    }
                }

                ImGui.EndCombo();
            }

            ImGui.SliderFloat("Damping##S2", ref _tuning2.StretchDamping, 0.0f, 4.0f, "%.1f");
            ImGui.SliderFloat("Hertz##S2", ref _tuning2.StretchHertz, 0.0f, 60.0f, "%.0f");
            ImGui.SliderFloat("Stiffness##S2", ref _tuning2.StretchStiffness, 0.0f, 1.0f, "%.1f");

            ImGui.SliderInt("Iterations##2", ref _iterations2, 1, 100, "%d");

            ImGui.Separator();

            ImGui.SliderFloat("Speed", ref _speed, 10.0f, 100.0f, "%.0f");

            if (ImGui.Button("Reset"))
            {
                _position1.Set(-5.0f, 15.0f);
                _position2.Set(5.0f, 15.0f);
                _rope1.Reset(_position1);
                _rope2.Reset(_position2);
            }

            ImGui.PopItemWidth();

            ImGui.End();

            _rope1.Draw(Drawer);
            _rope2.Draw(Drawer);

            DrawString("Press comma and period to move left and right");
        }

        protected override void PreStep()
        {
            var dt = TestSettings.Hertz > 0.0f ? 1.0f / TestSettings.Hertz : 0.0f;
            if (Game.IsKeyDown(Key.Comma))
            {
                _position1.X -= _speed * dt;
                _position2.X -= _speed * dt;
            }

            if (Game.IsKeyDown(Key.Period))
            {
                _position1.X += _speed * dt;
                _position2.X += _speed * dt;
            }

            if (TestSettings.Pause && !TestSettings.SingleStep)
            {
                dt = 0.0f;
            }

            _rope1.SetTuning(_tuning1);
            _rope2.SetTuning(_tuning2);
            _rope1.Step(dt, _iterations1, _position1);
            _rope2.Step(dt, _iterations2, _position2);
        }
    }
}