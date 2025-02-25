using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Joints;

[Sample("Joints", "Breakable Joint")]
public class BreakableJoint : SampleBase
{
    public const int e_count = 6;

    public BreakableJoint(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 8.0f);
            Global.Camera.Zoom = 25.0f * 0.7f;
        }

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        BodyId groundId = Body.CreateBody(WorldId, bodyDef);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        Segment segment = ((-40.0f, 0.0f), (40.0f, 0.0f));
        Shape.CreateSegmentShape(groundId, shapeDef, segment);

        for (int i = 0; i < e_count; ++i)
        {
            m_jointIds[i] = JointId.NullId;
        }

        Vec2 position = (-12.5f, 10.0f);
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.EnableSleep = false;

        Polygon box = Geometry.MakeBox(1.0f, 1.0f);

        int index = 0;

        // distance joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            float length = 2.0f;
            Vec2 pivot1 = (position.X, position.Y + 1.0f + length);
            Vec2 pivot2 = (position.X, position.Y + 1.0f);
            DistanceJointDef jointDef = DistanceJointDef.DefaultDistanceJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot1);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot2);
            jointDef.Length = length;
            jointDef.CollideConnected = true;
            m_jointIds[index] = Joint.CreateDistanceJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;

        // motor joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            MotorJointDef jointDef = MotorJointDef.DefaultMotorJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.LinearOffset = position;
            jointDef.MaxForce = 1000.0f;
            jointDef.MaxTorque = 20.0f;
            jointDef.CollideConnected = true;
            m_jointIds[index] = Joint.CreateMotorJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;

        // prismatic joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            Vec2 pivot = (position.X - 1.0f, position.Y);
            PrismaticJointDef jointDef = PrismaticJointDef.DefaultPrismaticJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.LocalAxisA = Body.GetLocalVector(jointDef.BodyIdA, (1.0f, 0.0f));
            jointDef.CollideConnected = true;
            m_jointIds[index] = Joint.CreatePrismaticJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;

        // revolute joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            Vec2 pivot = (position.X - 1.0f, position.Y);
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.CollideConnected = true;
            m_jointIds[index] = Joint.CreateRevoluteJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;

        // weld joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            Vec2 pivot = (position.X - 1.0f, position.Y);
            WeldJointDef jointDef = WeldJointDef.DefaultWeldJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.AngularHertz = 2.0f;
            jointDef.AngularDampingRatio = 0.5f;
            jointDef.LinearHertz = 2.0f;
            jointDef.LinearDampingRatio = 0.5f;
            jointDef.CollideConnected = true;
            m_jointIds[index] = Joint.CreateWeldJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;

        // wheel joint
        {
            Debug.Assert(index < e_count);

            bodyDef.Position = position;
            BodyId bodyId = Body.CreateBody(WorldId, bodyDef);
            Shape.CreatePolygonShape(bodyId, shapeDef, box);

            Vec2 pivot = (position.X - 1.0f, position.Y);
            WheelJointDef jointDef = WheelJointDef.DefaultWheelJointDef();
            jointDef.BodyIdA = groundId;
            jointDef.BodyIdB = bodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.LocalAxisA = Body.GetLocalVector(jointDef.BodyIdA, (1.0f, 0.0f));
            jointDef.Hertz = 1.0f;
            jointDef.DampingRatio = 0.7f;
            jointDef.LowerTranslation = -1.0f;
            jointDef.UpperTranslation = 1.0f;
            jointDef.EnableLimit = true;
            jointDef.EnableMotor = true;
            jointDef.MaxMotorTorque = 10.0f;
            jointDef.MotorSpeed = 1.0f;
            jointDef.CollideConnected = true;
            m_jointIds[index] = Joint.CreateWheelJoint(WorldId, jointDef);
        }

        position.X += 5.0f;
        ++index;

        m_breakForce = 1000.0f;
    }

    protected override void OnRender()
    {
        base.OnRender();

        for (int i = 0; i < e_count; ++i)
        {
            if ((m_jointIds[i]).IsNull)
            {
                continue;
            }

            Vec2 force = Joint.GetConstraintForce(m_jointIds[i]);
            if (B2Math.LengthSquared(force) > m_breakForce * m_breakForce)
            {
                Joint.DestroyJoint(m_jointIds[i]);
                m_jointIds[i] = JointId.NullId;
            }
            else
            {
                Vec2 point = Joint.GetLocalAnchorA(m_jointIds[i]);
                Draw.DrawString(point, $"({force.X:F1}, {force.Y:F1})");
            }
        }
    }

    protected JointId[] m_jointIds = new JointId[e_count];

    protected float m_breakForce;
}