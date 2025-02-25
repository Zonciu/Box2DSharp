using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Benchmark;

[Sample("Benchmark", "Kinematic")]
public class Kinematic : SampleBase
{
    public Kinematic(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 0.0f);
            Global.Camera.Zoom = 150.0f;
        }

        float grid = 1.0f;

        int span = Core.B2Debug ? 20 : 100;

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.KinematicBody;
        bodyDef.AngularVelocity = 1.0f;

        // defer mass properties to avoid n-squared mass computations
        bodyDef.AutomaticMass = false;

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Filter.CategoryBits = 1;
        shapeDef.Filter.MaskBits = 2;

        BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

        for (int i = -span; i < span; ++i)
        {
            float y = i * grid;
            for (int j = -span; j < span; ++j)
            {
                float x = j * grid;
                Polygon square = Geometry.MakeOffsetBox(0.5f * grid, 0.5f * grid, (x, y), Rot.Identity);
                Shape.CreatePolygonShape(bodyId, shapeDef, square);
            }
        }

        // All shapes have been added so I can efficiently compute the mass properties.
        Body.ApplyMassFromShapes(bodyId);
    }
}