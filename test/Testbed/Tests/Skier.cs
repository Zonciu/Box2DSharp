using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Box2DSharp.Collision.Shapes;
using Box2DSharp.Common;
using Box2DSharp.Dynamics;
using OpenToolkit.Windowing.Common;
using OpenToolkit.Windowing.Common.Input;
using Testbed.Basics;

namespace Testbed.Tests
{
    [TestCase("Bugs", "Skier")]
    public class Skier : Test
    {
        public Skier()
        {
            Body ground = null;
            {
                BodyDef bd = new BodyDef();
                ground = World.CreateBody(bd);

                const float PlatformWidth = 8.0f;

                /*
                First angle is from the horizontal and should be negative for a downward slope.
                Second angle is relative to the preceding slope, and should be positive, creating a kind of
                loose 'Z'-shape from the 3 edges.
                If A1 = -10, then A2 <= ~1.5 will result in the collision glitch.
                If A1 = -30, then A2 <= ~10.0 will result in the glitch.
                */
                const float Angle1Degrees = -30.0f;
                const float Angle2Degrees = 10.0f;

                /*
                The larger the value of SlopeLength, the less likely the glitch will show up.
                */
                const float SlopeLength = 2.0f;

                const float SurfaceFriction = 0.2f;

                // Convert to radians
                const float Slope1Incline = -Angle1Degrees * Settings.Pi / 180.0f;
                const float Slope2Incline = Slope1Incline - Angle2Degrees * Settings.Pi / 180.0f;

                //

                Platform_width = PlatformWidth;

                List<Vector2> verts = new List<Vector2>();

                // Horizontal platform
                verts.Add(new Vector2(-PlatformWidth, 0.0f));
                verts.Add(Vector2.Zero);

                // Slope
                verts.Add(
                    new Vector2(
                        verts.Last().X + SlopeLength * (float)Math.Cos(Slope1Incline),
                        verts.Last().Y - SlopeLength * (float)Math.Sin(Slope1Incline)
                    ));

                verts.Add(
                    new Vector2(
                        verts.Last().X + SlopeLength * (float)Math.Cos(Slope2Incline),
                        verts.Last().Y - SlopeLength * (float)Math.Sin(Slope2Incline)
                    ));

                {
                    EdgeShape shape = new EdgeShape();
                    shape.Set(verts[0], verts[1]);
                    shape.HasVertex3 = true;
                    shape.Vertex3 = verts[2];

                    FixtureDef fd = new FixtureDef();
                    fd.Shape = shape;
                    fd.Density = 0.0f;
                    fd.Friction = SurfaceFriction;

                    ground.CreateFixture(fd);
                }

                {
                    EdgeShape shape = new EdgeShape();
                    shape.Set(verts[1], verts[2]);
                    shape.HasVertex0 = true;
                    shape.HasVertex3 = true;
                    shape.Vertex0 = verts[0];
                    shape.Vertex3 = verts[3];

                    FixtureDef fd = new FixtureDef();
                    fd.Shape = shape;
                    fd.Density = 0.0f;
                    fd.Friction = SurfaceFriction;

                    ground.CreateFixture(fd);
                }

                {
                    EdgeShape shape = new EdgeShape();
                    shape.Set(verts[2], verts[3]);
                    shape.HasVertex0 = true;
                    shape.Vertex0 = verts[1];

                    FixtureDef fd = new FixtureDef();
                    fd.Shape = shape;
                    fd.Density = 0.0f;
                    fd.Friction = SurfaceFriction;

                    ground.CreateFixture(fd);
                }
            }

            {
                const bool EnableCircularSkiTips = false;

                const float BodyWidth = 1.0f;
                const float BodyHeight = 2.5f;
                const float SkiLength = 3.0f;

                /*
                Larger values for this seem to alleviate the issue to some extent.
                */
                const float SkiThickness = 0.3f;

                const float SkiFriction = 0.0f;
                const float SkiRestitution = 0.15f;

                BodyDef bd = new BodyDef();
                bd.BodyType = BodyType.DynamicBody;

                float initial_y = BodyHeight / 2 + SkiThickness;
                if (EnableCircularSkiTips)
                {
                    initial_y += SkiThickness / 6;
                }

                bd.Position.Set(-Platform_width / 2, initial_y);

                Body skier = World.CreateBody(bd);

                PolygonShape body = new PolygonShape();
                body.SetAsBox(BodyWidth / 2, BodyHeight / 2);

                PolygonShape ski = new PolygonShape();
                List<Vector2> verts = new List<Vector2>();
                verts.Add(new Vector2(-SkiLength / 2 - SkiThickness, -BodyHeight / 2));
                verts.Add(new Vector2(-SkiLength / 2, -BodyHeight / 2 - SkiThickness));
                verts.Add(new Vector2(SkiLength / 2, -BodyHeight / 2 - SkiThickness));
                verts.Add(new Vector2(SkiLength / 2 + SkiThickness, -BodyHeight / 2));
                ski.Set(verts.ToArray());

                CircleShape ski_back_shape = new CircleShape();
                ski_back_shape.Position.Set(-SkiLength / 2.0f, -BodyHeight / 2 - SkiThickness * (2.0f / 3));
                ski_back_shape.Radius = SkiThickness / 2;

                CircleShape ski_front_shape = new CircleShape();
                ski_front_shape.Position.Set(SkiLength / 2, -BodyHeight / 2 - SkiThickness * (2.0f / 3));
                ski_front_shape.Radius = SkiThickness / 2;

                FixtureDef fd = new FixtureDef();
                fd.Shape = body;
                fd.Density = 1.0f;
                skier.CreateFixture(fd);

                fd.Friction = SkiFriction;
                fd.Restitution = SkiRestitution;

                fd.Shape = ski;
                skier.CreateFixture(fd);

                if (EnableCircularSkiTips)
                {
                    fd.Shape = ski_back_shape;
                    skier.CreateFixture(fd);

                    fd.Shape = ski_front_shape;
                    skier.CreateFixture(fd);
                }

                skier.SetLinearVelocity(new Vector2(0.5f, 0.0f));

                _skier = skier;
            }

            Global.Camera.Center = new Vector2(Platform_width / 2.0f, 0.0f);
            Global.Camera.Zoom = 0.4f;
            Fixed_camera = true;
        }

        /// <inheritdoc />
        public override void OnKeyDown(KeyboardKeyEventArgs key)
        {
            switch (key.Key)
            {
            case Key.C:
                Fixed_camera = !Fixed_camera;
                if (Fixed_camera)
                {
                    Global.Camera.Center = new Vector2(Platform_width / 2.0f, 0.0f);
                }

                break;
            }
        }

        protected override void OnRender()
        {
            DrawString("Keys: c = Camera fixed/tracking");

            if (!Fixed_camera)
            {
                Global.Camera.Center = _skier.GetPosition();
            }
        }

        Body _skier;

        float Platform_width;

        bool Fixed_camera;
    }
}