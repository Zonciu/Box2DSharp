using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Events;

/// <summary>
/// Shows how to make a rigid body character mover and use the pre-solve callback.
/// </summary>
[Sample("Events", "Platformer")]
public class Platformer : SampleBase
{
    protected bool _canJump;

    bool m_jumping;

    float m_radius;

    protected float m_force;

    protected float m_impulse;

    float m_jumpDelay;

    BodyId m_characterId;

    BodyId m_platformId;

    ShapeId m_platformShapeId;

    public Platformer(Settings settings)
        : base(settings)
    {
        if (Settings.Restart == false)
        {
            Global.Camera.Center = (0.5f, 7.5f);
            Global.Camera.Zoom = 25.0f * 0.4f;
        }

        World.SetPreSolveCallback(WorldId, PreSolveStatic, this);

        // Ground
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Segment segment = ((-20.0f, 0.0f), (20.0f, 0.0f));
            Shape.CreateSegmentShape(groundId, shapeDef, segment);
        }

        // Platform
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.KinematicBody;
            bodyDef.Position = (0.0f, 6.0f);
            bodyDef.LinearVelocity = (2.0f, 0.0f);
            m_platformId = Body.CreateBody(WorldId, bodyDef);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            Polygon box = Geometry.MakeBox(3.0f, 0.5f);
            m_platformShapeId = Shape.CreatePolygonShape(m_platformId, shapeDef, box);
        }

        // Actor
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.FixedRotation = true;
            bodyDef.LinearDamping = 0.5f;
            bodyDef.Position = (0.0f, 1.0f);
            m_characterId = Body.CreateBody(WorldId, bodyDef);

            m_radius = 0.5f;
            Capsule capsule = ((0.0f, 0.0f), (0.0f, 1.0f), m_radius);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.Friction = 0.1f;

            // Need to turn this on to get the callback
            shapeDef.EnablePreSolveEvents = true;

            Shape.CreateCapsuleShape(m_characterId, shapeDef, capsule);
        }

        m_force = 25.0f;
        m_impulse = 25.0f;
        m_jumpDelay = 0.25f;
        m_jumping = false;
    }

    static bool PreSolveStatic(ShapeId shapeIdA, ShapeId shapeIdB, ref Manifold manifold, object context)
    {
        Platformer platformer = (Platformer)context;
        return platformer.PreSolve(shapeIdA, shapeIdB, manifold);
    }

    // This callback must be thread-safe. It may be called multiple times simultaneously.
    // Notice how this method is constant and doesn't change any data. It also
    // does not try to access any values in the world that may be changing, such as contact data.
    bool PreSolve(in ShapeId shapeIdA, in ShapeId shapeIdB, in Manifold manifold)
    {
        Debug.Assert(World.ShapeIsValid(shapeIdA));
        Debug.Assert(World.ShapeIsValid(shapeIdB));

        ShapeId actorShapeId;
        float sign = 0.0f;
        if (shapeIdA == m_platformShapeId)
        {
            sign = 1.0f;
            actorShapeId = shapeIdB;
        }
        else if (shapeIdB == m_platformShapeId)
        {
            sign = -1.0f;
            actorShapeId = shapeIdA;
        }
        else
        {
            // not the platform, enable contact
            return true;
        }

        BodyId bodyId = Shape.GetBody(actorShapeId);
        if (bodyId != m_characterId)
        {
            // not the character, enable contact
            return true;
        }

        Vec2 normal = manifold.Normal;
        if (sign * normal.Y > 0.95f)
        {
            return true;
        }

        float separation = 0.0f;
        for (int i = 0; i < manifold.PointCount; ++i)
        {
            float s = manifold.Points[i].Separation;
            separation = separation < s ? separation : s;
        }

        if (separation > 0.1f * m_radius)
        {
            // shallow overlap
            return true;
        }

        // normal points down, disable contact
        return false;
    }

 
    public override void Step()
    {
        _canJump = false;
        Vec2 velocity = Body.GetLinearVelocity(m_characterId);
        if (m_jumpDelay == 0.0f && m_jumping == false && velocity.Y < 0.01f)
        {
            int capacity = Body.GetContactCapacity(m_characterId);
            capacity = Math.Min(capacity, 4);
            ContactData[] contactData = new ContactData[4];
            int count = Body.GetContactData(m_characterId, contactData, capacity);
            for (int i = 0; i < count; ++i)
            {
                BodyId bodyIdA = Shape.GetBody(contactData[i].ShapeIdA);
                float sign = 0.0f;
                if (bodyIdA == m_characterId)
                {
                    // normal points from A to B
                    sign = -1.0f;
                }
                else
                {
                    sign = 1.0f;
                }

                if (sign * contactData[i].Manifold.Normal.Y > 0.9f)
                {
                    _canJump = true;
                    break;
                }
            }
        }

        // A kinematic body is moved by setting its velocity. This
        // ensure friction works correctly.
        Vec2 platformPosition = Body.GetPosition(m_platformId);
        if (platformPosition.X < -15.0f)
        {
            Body.SetLinearVelocity(m_platformId, (2.0f, 0.0f));
        }
        else if (platformPosition.X > 15.0f)
        {
            Body.SetLinearVelocity(m_platformId, (-2.0f, 0.0f));
        }

        if (Input.IsKeyDown(KeyCodes.A))
        {
            Body.ApplyForceToCenter(m_characterId, (-m_force, 0.0f), true);
        }

        if (Input.IsKeyDown(KeyCodes.D))
        {
            Body.ApplyForceToCenter(m_characterId, (m_force, 0.0f), true);
        }

        if (Input.IsKeyDown(KeyCodes.Space))
        {
            if (_canJump)
            {
                Body.ApplyLinearImpulseToCenter(m_characterId, (0.0f, m_impulse), true);
                m_jumpDelay = 0.5f;
                m_jumping = true;
            }
        }
        else
        {
            m_jumping = false;
        }

        base.Step();
        if (Settings.Hertz > 0.0f)
        {
            m_jumpDelay = Math.Max(0.0f, m_jumpDelay - 1.0f / Settings.Hertz);
        }
    }
}