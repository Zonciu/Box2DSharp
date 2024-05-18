using Box2DSharp.Dynamics.Joints;
using ImGuiNET;
using Testbed.TestCases;
using UnityEngine;

namespace Box2DSharp.Testbed.Unity.Tests
{
    [TestInherit]
    public class DistanceJointTestRender : DistanceJointTest
    {
        /// <inheritdoc />
        protected override void OnRender()
        {
            ImGui.SetNextWindowPos(new Vector2(10.0f, 100.0f));
            ImGui.SetNextWindowSize(new Vector2(260.0f, 150.0f));
            ImGui.Begin("Joint Controls", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize);

            if (ImGui.SliderFloat("Length", ref m_length, 0.0f, 20.0f, "%.0f"))
            {
                m_length = m_joint.SetLength(m_length);
            }

            if (ImGui.SliderFloat("Min Length", ref m_minLength, 0.0f, 20.0f, "%.0f"))
            {
                m_minLength = m_joint.SetMinLength(m_minLength);
            }

            if (ImGui.SliderFloat("Max Length", ref m_maxLength, 0.0f, 20.0f, "%.0f"))
            {
                m_maxLength = m_joint.SetMaxLength(m_maxLength);
            }

            if (ImGui.SliderFloat("Hertz", ref m_hertz, 0.0f, 10.0f, "%.1f"))
            {
                JointUtils.LinearStiffness(out var stiffness, out var damping, m_hertz, m_dampingRatio, m_joint.BodyA, m_joint.BodyB);
                m_joint.Stiffness = stiffness;
                m_joint.Damping = damping;
            }

            if (ImGui.SliderFloat("Damping Ratio", ref m_dampingRatio, 0.0f, 2.0f, "%.1f"))
            {
                JointUtils.LinearStiffness(out var stiffness, out var damping, m_hertz, m_dampingRatio, m_joint.BodyA, m_joint.BodyB);
                m_joint.Stiffness = stiffness;
                m_joint.Damping = damping;
            }

            ImGui.End();
            base.OnRender();
        }
    }
}