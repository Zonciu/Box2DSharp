using Box2DSharp;
using ImGuiNET;
using Testbed.Abstractions;

namespace Testbed.TestSamples.Joints;

/// <summary>
/// Test the distance joint and all options
/// </summary>
[TestInherit]
public class DistanceJointSample : Samples.Joints.DistanceJointSample
{
    public DistanceJointSample(Settings settings)
        : base(settings)
    { }

    public override void UpdateUI()
    {
        float height = 140.0f;
        ImGui.SetNextWindowPos(new(10.0f, Global.Camera.Height - height - 50.0f), ImGuiCond.Once);
        ImGui.SetNextWindowSize(new(180.0f, height));

        ImGui.Begin("Distance Joint", ImGuiWindowFlags.NoResize);
        ImGui.PushItemWidth(100.0f);

        if (ImGui.SliderFloat("Length", ref m_length, 0.1f, 4.0f, "%3.1f"))
        {
            for (int i = 0; i < m_count; ++i)
            {
                DistanceJointFunc.SetLength(m_jointIds[i], m_length);
                Joint.WakeBodies(m_jointIds[i]);
            }
        }

        if (ImGui.Checkbox("Spring", ref m_enableSpring))
        {
            for (int i = 0; i < m_count; ++i)
            {
                DistanceJointFunc.EnableSpring(m_jointIds[i], m_enableSpring);
                Joint.WakeBodies(m_jointIds[i]);
            }
        }

        if (m_enableSpring)
        {
            if (ImGui.SliderFloat("Hertz", ref m_hertz, 0.0f, 15.0f, "%3.1f"))
            {
                for (int i = 0; i < m_count; ++i)
                {
                    DistanceJointFunc.SetSpringHertz(m_jointIds[i], m_hertz);
                    Joint.WakeBodies(m_jointIds[i]);
                }
            }

            if (ImGui.SliderFloat("Damping", ref m_dampingRatio, 0.0f, 4.0f, "%3.1f"))
            {
                for (int i = 0; i < m_count; ++i)
                {
                    DistanceJointFunc.SetSpringDampingRatio(m_jointIds[i], m_dampingRatio);
                    Joint.WakeBodies(m_jointIds[i]);
                }
            }
        }

        if (ImGui.Checkbox("Limit", ref m_enableLimit))
        {
            for (int i = 0; i < m_count; ++i)
            {
                DistanceJointFunc.EnableLimit(m_jointIds[i], m_enableLimit);
                Joint.WakeBodies(m_jointIds[i]);
            }
        }

        if (m_enableLimit)
        {
            if (ImGui.SliderFloat("Min Length", ref m_minLength, 0.1f, 4.0f, "%3.1f"))
            {
                for (int i = 0; i < m_count; ++i)
                {
                    DistanceJointFunc.SetLengthRange(m_jointIds[i], m_minLength, m_maxLength);
                    Joint.WakeBodies(m_jointIds[i]);
                }
            }

            if (ImGui.SliderFloat("Max Length", ref m_maxLength, 0.1f, 4.0f, "%3.1f"))
            {
                for (int i = 0; i < m_count; ++i)
                {
                    DistanceJointFunc.SetLengthRange(m_jointIds[i], m_minLength, m_maxLength);
                    Joint.WakeBodies(m_jointIds[i]);
                }
            }
        }

        int count = m_count;
        if (ImGui.SliderInt("Count", ref count, 1, e_maxCount))
        {
            CreateScene(count);
        }

        ImGui.PopItemWidth();
        ImGui.End();
    }
};

// A suspension bridge

// This sample shows the limitations of an iterative solver. The cantilever sags even though the weld
// joint is stiff as possible.

// This test ensures joints work correctly with bodies that have fixed rotation

// This sample shows how to break joints when the internal reaction force becomes large.

// This is a fun demo that shows off the wheel joint