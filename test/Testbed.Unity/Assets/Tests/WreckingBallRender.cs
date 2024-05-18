using ImGuiNET;
using Testbed.TestCases;
using UnityEngine;

namespace Box2DSharp.Testbed.Unity.Tests
{
    [TestInherit]
    public class WreckingBallRender : WreckingBall
    {
        /// <inheritdoc />
        protected override void OnRender()
        {
            base.OnRender();
            ImGui.SetNextWindowPos(new Vector2(10.0f, 100.0f));
            ImGui.SetNextWindowSize(new Vector2(200.0f, 100.0f));
            ImGui.Begin("Wrecking Ball Controls", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

            if (ImGui.Checkbox("Stabilize", ref _stabilize))
            {
                if (_stabilize && _distanceJoint == null)
                {
                    _distanceJoint = World.CreateJoint(_distanceJointDef);
                }
                else if (_stabilize == false && _distanceJoint != null)
                {
                    World.DestroyJoint(_distanceJoint);
                    _distanceJoint = null;
                }
            }

            ImGui.End();
        }
    }
}