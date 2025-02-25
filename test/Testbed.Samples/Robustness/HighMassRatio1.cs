using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Robustness;

[Sample("Robustness", "HighMassRatio1")]
public class HighMassRatio1 : SampleBase
{
    public HighMassRatio1(Settings settings)
        : base(settings)
    {
        if (settings.Restart == false)
        {
            Global.Camera.Center = (3.0f, 14.0f);
            Global.Camera.Zoom = 25.0f;
        }

        float extent = 1.0f;

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeOffsetBox(50.0f, 1.0f, (0.0f, -1.0f), Rot.Identity);
            Shape.CreatePolygonShape(groundId, shapeDef, box);
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            Polygon box = Geometry.MakeBox(extent, extent);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            for (int j = 0; j < 3; ++j)
            {
                int count = 10;
                float offset = -20.0f * extent + 2.0f * (count + 1.0f) * extent * j;
                float y = extent;
                while (count > 0)
                {
                    for (int i = 0; i < count; ++i)
                    {
                        float coeff = i - 0.5f * count;

                        float yy = count == 1 ? y + 2.0f : y;
                        bodyDef.Position = (2.0f * coeff * extent + offset, yy);
                        BodyId bodyId = Body.CreateBody(WorldId, bodyDef);

                        shapeDef.Density = count == 1 ? (j + 1.0f) * 100.0f : 1.0f;
                        Shape.CreatePolygonShape(bodyId, shapeDef, box);
                    }

                    --count;
                    y += 2.0f * extent;
                }
            }
        }
    }
}

// Big box on small boxes

// This sample shows how careful creation of compound shapes leads to better simulation and avoids
// objects getting stuck.
// This also shows how to get the combined AABB for the body.

// This shows how to use custom filtering

// Restitution is approximate since Box2D uses speculative collision

// This sample shows how to modify the geometry on an existing shape. This is only supported on
// dynamic and kinematic shapes because static shapes don't look for new collisions.

// Shows how to link to chain shapes together. This is a useful technique for building large game levels with smooth collision.

// This shows how to handle high gravity and small shapes using a small time step

// From PEEL