using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Benchmark;

[Sample("Benchmark", "Tumbler")]
public class Tumbler(Settings settings) : SampleBase(settings)
{
    protected JointId JointId;

    protected float MotorSpeed;

    public override void OnInitialized()
    {
        if (Settings.Restart == false)
        {
            Global.Camera.Center = (1.5f, 10.0f);
            Global.Camera.Zoom = 25.0f * 0.6f;
        }

        BodyId groundId;
        {
            var bodyDef = BodyDef.DefaultBodyDef();
            groundId = Body.CreateBody(WorldId, bodyDef);
        }

        {
            var bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.EnableSleep = true;
            bodyDef.Position = (0.0f, 10.0f);
            var bodyId = Body.CreateBody(WorldId, bodyDef);

            var shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Density = 50.0f;

            Polygon polygon;
            polygon = Geometry.MakeOffsetBox(
                0.5f,
                10.0f,
                (10.0f, 0.0f),
                Rot.Identity);
            Shape.CreatePolygonShape(bodyId, shapeDef, polygon);
            polygon = Geometry.MakeOffsetBox(
                0.5f,
                10.0f,
                (-10.0f, 0.0f),
                Rot.Identity);
            Shape.CreatePolygonShape(bodyId, shapeDef, polygon);
            polygon = Geometry.MakeOffsetBox(
                10.0f,
                0.5f,
                (0.0f, 10.0f),
                Rot.Identity);
            Shape.CreatePolygonShape(bodyId, shapeDef, polygon);
            polygon = Geometry.MakeOffsetBox(
                10.0f,
                0.5f,
                (0.0f, -10.0f),
                Rot.Identity);
            Shape.CreatePolygonShape(bodyId, shapeDef, polygon);

            // m_motorSpeed = 9.0f;
            MotorSpeed = 25.0f;

            var jd = RevoluteJointDef.DefaultRevoluteJointDef();
            jd.BodyIdA = groundId;
            jd.BodyIdB = bodyId;
            jd.LocalAnchorA = (0.0f, 10.0f);
            jd.LocalAnchorB = (0.0f, 0.0f);
            jd.ReferenceAngle = 0.0f;
            jd.MotorSpeed = B2Math.Pi / 180.0f * MotorSpeed;
            jd.MaxMotorTorque = 1e8f;
            jd.EnableMotor = true;

            JointId = Joint.CreateRevoluteJoint(WorldId, jd);
        }
        {
            var gridCount = Core.B2Debug ? 20 : 45;
            var polygon = Geometry.MakeBox(0.125f, 0.125f);
            var bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            var shapeDef = ShapeDef.DefaultShapeDef();

            var y = -0.2f * gridCount + 10.0f;
            for (var i = 0; i < gridCount; ++i)
            {
                var x = -0.2f * gridCount;

                for (var j = 0; j < gridCount; ++j)
                {
                    bodyDef.Position = (x, y);
                    var bodyId = Body.CreateBody(WorldId, bodyDef);

                    Shape.CreatePolygonShape(bodyId, shapeDef, polygon);

                    x += 0.4f;
                }

                y += 0.4f;
            }
        }
    }
}