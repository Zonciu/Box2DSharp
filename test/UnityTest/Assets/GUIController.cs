using System;
using Box2DSharp.Testbed.Unity.Inspection;
using ImGuiNET;
using Testbed.Abstractions;
using UnityEngine;

namespace Box2DSharp.Testbed.Unity
{
    public class GUIController
    {
        public readonly Game _game;

        public GUIController(Game game)
        {
            _game = game;
        }

        public void Render()
        {
            UpdateText();
            UpdateUI();
        }

        private readonly Vector4 _textColor = new Vector4(0.9f, 0.6f, 0.6f, 1);

        private void UpdateText()
        {
            if (_game.DebugDrawer.ShowUI)
            {
                ImGui.SetNextWindowPos(new Vector2(0.0f, 0.0f));
                ImGui.SetNextWindowSize(new Vector2(Global.Camera.Width, Global.Camera.Height));
                ImGui.SetNextWindowBgAlpha(0);
                ImGui.Begin("Overlay", ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoScrollbar);
                ImGui.End();
                var (category, name, _) = Global.Tests[Global.Settings.TestIndex];
                _game.Test?.DrawTitle($"{category} : {name}");

                while (_game.DebugDrawer.Texts.TryDequeue(out var text))
                {
                    ImGui.Begin("Overlay", ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoScrollbar);
                    ImGui.SetCursorPos(text.Position.ToUnityVector2());
                    ImGui.TextColored(_textColor, text.Text);
                    ImGui.End();
                }
            }
        }

        public void UpdateUI()
        {
            const int MenuWidth = 180;
            if (_game.DebugDrawer.ShowUI)
            {
                _game.Test.Render();
                ImGui.SetNextWindowPos(new Vector2((float)Global.Camera.Width - MenuWidth - 10, 10));
                ImGui.SetNextWindowSize(new Vector2(MenuWidth, (float)Global.Camera.Height - 20));

                ImGui.Begin("Tools", ref _game.DebugDrawer.ShowUI, ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse);

                if (ImGui.BeginTabBar("ControlTabs", ImGuiTabBarFlags.None))
                {
                    if (ImGui.BeginTabItem("Controls"))
                    {
                        ImGui.SliderInt("Vel Iters", ref Global.Settings.VelocityIterations, 0, 50);
                        ImGui.SliderInt("Pos Iters", ref Global.Settings.PositionIterations, 0, 50);
                        ImGui.SliderFloat("Hertz", ref Global.Settings.Hertz, 5.0f, 120.0f, "%.0f hz");

                        ImGui.Separator();

                        ImGui.Checkbox("Sleep", ref Global.Settings.EnableSleep);
                        ImGui.Checkbox("Warm Starting", ref Global.Settings.EnableWarmStarting);
                        ImGui.Checkbox("Time of Impact", ref Global.Settings.EnableContinuous);
                        ImGui.Checkbox("Sub-Stepping", ref Global.Settings.EnableSubStepping);

                        ImGui.Separator();

                        ImGui.Checkbox("Shapes", ref Global.Settings.DrawShapes);
                        ImGui.Checkbox("Joints", ref Global.Settings.DrawJoints);
                        ImGui.Checkbox("AABBs", ref Global.Settings.DrawAABBs);
                        ImGui.Checkbox("Contact Points", ref Global.Settings.DrawContactPoints);
                        ImGui.Checkbox("Contact Normals", ref Global.Settings.DrawContactNormals);
                        ImGui.Checkbox("Contact Impulses", ref Global.Settings.DrawContactImpulse);
                        ImGui.Checkbox("Friction Impulses", ref Global.Settings.DrawFrictionImpulse);
                        ImGui.Checkbox("Center of Masses", ref Global.Settings.DrawCOMs);
                        ImGui.Checkbox("Statistics", ref Global.Settings.DrawStats);
                        ImGui.Checkbox("Profile", ref Global.Settings.DrawProfile);

                        var buttonSz = new Vector2(-1, 0);
                        if (ImGui.Button("Pause (P)", buttonSz))
                        {
                            Global.Settings.Pause = !Global.Settings.Pause;
                        }

                        if (ImGui.Button("Single Step (O)", buttonSz))
                        {
                            Global.Settings.SingleStep = !Global.Settings.SingleStep;
                        }

                        if (ImGui.Button("Restart (R)", buttonSz))
                        {
                            _game.RestartTest();
                        }

                        if (ImGui.Button("Quit", buttonSz))
                        {
                            Application.Quit();
                        }

                        ImGui.EndTabItem();
                    }

                    var leafNodeFlags = ImGuiTreeNodeFlags.OpenOnArrow | ImGuiTreeNodeFlags.OpenOnDoubleClick;
                    leafNodeFlags |= ImGuiTreeNodeFlags.Leaf | ImGuiTreeNodeFlags.NoTreePushOnOpen;

                    const ImGuiTreeNodeFlags NodeFlags = ImGuiTreeNodeFlags.OpenOnArrow | ImGuiTreeNodeFlags.OpenOnDoubleClick;

                    if (ImGui.BeginTabItem("Tests"))
                    {
                        var categoryIndex = 0;
                        var category = Global.Tests[categoryIndex].Category;
                        var i = 0;
                        while (i < Global.Tests.Count)
                        {
                            var categorySelected = string.CompareOrdinal(category, Global.Tests[Global.Settings.TestIndex].Category) == 0;
                            var nodeSelectionFlags = categorySelected ? ImGuiTreeNodeFlags.Selected : 0;
                            var nodeOpen = ImGui.TreeNodeEx(category, NodeFlags | nodeSelectionFlags);

                            if (nodeOpen)
                            {
                                while (i < Global.Tests.Count && string.CompareOrdinal(category, Global.Tests[i].Category) == 0)
                                {
                                    ImGuiTreeNodeFlags selectionFlags = 0;
                                    if (Global.Settings.TestIndex == i)
                                    {
                                        selectionFlags = ImGuiTreeNodeFlags.Selected;
                                    }

                                    ImGui.TreeNodeEx((IntPtr)i, leafNodeFlags | selectionFlags, Global.Tests[i].Name);
                                    if (ImGui.IsItemClicked())
                                    {
                                        _game.SetTest(i);
                                    }

                                    ++i;
                                }

                                ImGui.TreePop();
                            }
                            else
                            {
                                while (i < Global.Tests.Count && string.CompareOrdinal(category, Global.Tests[i].Category) == 0)
                                {
                                    ++i;
                                }
                            }

                            if (i < Global.Tests.Count)
                            {
                                category = Global.Tests[i].Category;
                                categoryIndex = i;
                            }
                        }

                        ImGui.EndTabItem();
                    }

                    ImGui.EndTabBar();
                }

                ImGui.End();
            }
        }
    }
}