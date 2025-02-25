using System.Diagnostics;
using Box2DSharp;
using Testbed.Abstractions;

namespace Testbed.Samples.Events;

[Sample("Events", "Contact")]
public class ContactEvent : SampleBase
{
    protected const int e_count = 20;

    class BodyUserData
    {
        public int Index;
    }

    public ContactEvent(Settings settings)
        : base(settings)
    {
        if (Settings.Restart == false)
        {
            Global.Camera.Center = (0.0f, 0.0f);
            Global.Camera.Zoom = 25.0f * 1.75f;
        }

        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            BodyId groundId = Body.CreateBody(WorldId, bodyDef);

            Vec2[] points = [(40.0f, -40.0f), (-40.0f, -40.0f), (-40.0f, 40.0f), (40.0f, 40.0f)];

            ChainDef chainDef = ChainDef.DefaultChainDef();
            chainDef.Count = 4;
            chainDef.Points = points;
            chainDef.IsLoop = true;

            Shape.CreateChain(groundId, chainDef);
        }

        // Player
        {
            BodyDef bodyDef = BodyDef.DefaultBodyDef();
            bodyDef.Type = BodyType.DynamicBody;
            bodyDef.GravityScale = 0.0f;
            bodyDef.LinearDamping = 0.5f;
            bodyDef.AngularDamping = 0.5f;
            bodyDef.IsBullet = true;
            m_playerId = Body.CreateBody(WorldId, bodyDef);

            Circle circle = ((0.0f, 0.0f), 1.0f);
            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();

            // Enable contact evts for the player shape
            shapeDef.EnableContactEvents = true;

            m_coreShapeId = Shape.CreateCircleShape(m_playerId, shapeDef, circle);
        }

        for (int i = 0; i < e_count; ++i)
        {
            m_debrisIds[i] = BodyId.NullId;
            m_bodyUserData[i].Index = i;
        }

        m_wait = 0.5f;
        m_force = 200.0f;
    }

    void SpawnDebris()
    {
        int index = -1;
        for (int i = 0; i < e_count; ++i)
        {
            if (m_debrisIds[i].IsNull)
            {
                index = i;
                break;
            }
        }

        if (index == -1)
        {
            return;
        }

        // Debris
        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.Position = (B2Random.Shared.RandomFloat(-38.0f, 38.0f), B2Random.Shared.RandomFloat(-38.0f, 38.0f));
        bodyDef.Rotation = B2Math.MakeRot(B2Random.Shared.RandomFloat(-B2Math.Pi, B2Math.Pi));
        bodyDef.LinearVelocity = (B2Random.Shared.RandomFloat(-5.0f, 5.0f), B2Random.Shared.RandomFloat(-5.0f, 5.0f));
        bodyDef.AngularVelocity = B2Random.Shared.RandomFloat(-1.0f, 1.0f);
        bodyDef.GravityScale = 0.0f;
        bodyDef.UserData = m_bodyUserData[index];
        m_debrisIds[index] = Body.CreateBody(WorldId, bodyDef);

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Restitution = 0.8f;

        // No evts when debris hits debris
        shapeDef.EnableContactEvents = false;

        if ((index + 1) % 3 == 0)
        {
            Circle circle = ((0.0f, 0.0f), 0.5f);
            Shape.CreateCircleShape(m_debrisIds[index], shapeDef, circle);
        }
        else if ((index + 1) % 2 == 0)
        {
            Capsule capsule = ((0.0f, -0.25f), (0.0f, 0.25f), 0.25f);
            Shape.CreateCapsuleShape(m_debrisIds[index], shapeDef, capsule);
        }
        else
        {
            Polygon box = Geometry.MakeBox(0.4f, 0.6f);
            Shape.CreatePolygonShape(m_debrisIds[index], shapeDef, box);
        }
    }

    public override void Step()
    {
        Vec2 position = Body.GetPosition(m_playerId);

        if (Input.IsKeyDown(KeyCodes.A))
        {
            Body.ApplyForce(m_playerId, (-m_force, 0.0f), position, true);
        }

        if (Input.IsKeyDown(KeyCodes.D))
        {
            Body.ApplyForce(m_playerId, (m_force, 0.0f), position, true);
        }

        if (Input.IsKeyDown(KeyCodes.W))
        {
            Body.ApplyForce(m_playerId, (0.0f, m_force), position, true);
        }

        if (Input.IsKeyDown(KeyCodes.S))
        {
            Body.ApplyForce(m_playerId, (0.0f, -m_force), position, true);
        }

        base.Step();

        // Discover rings that touch the bottom sensor
        int[] debrisToAttach = new int[e_count];
        ShapeId[] shapesToDestroy = new ShapeId[e_count];
        int attachCount = 0;
        int destroyCount = 0;

        ContactEvents contactEvents = World.GetContactEvents(WorldId);
        var beginEvents = contactEvents.BeginEvents.Span;
        for (int i = 0; i < contactEvents.BeginCount; ++i)
        {
            ref readonly ContactBeginTouchEvent evt = ref beginEvents[i];
            BodyId bodyIdA = Shape.GetBody(evt.ShapeIdA);
            BodyId bodyIdB = Shape.GetBody(evt.ShapeIdB);

            if (bodyIdA == m_playerId)
            {
                var userDataB = (BodyUserData?)Body.GetUserData(bodyIdB);
                if (userDataB == null)
                {
                    if (evt.ShapeIdA != m_coreShapeId && destroyCount < e_count)
                    {
                        // player non-core shape hit the wall

                        bool found = false;
                        for (int j = 0; j < destroyCount; ++j)
                        {
                            if (evt.ShapeIdA == shapesToDestroy[j])
                            {
                                found = true;
                                break;
                            }
                        }

                        // avoid double deletion
                        if (found == false)
                        {
                            shapesToDestroy[destroyCount] = evt.ShapeIdA;
                            destroyCount += 1;
                        }
                    }
                }
                else if (attachCount < e_count)
                {
                    debrisToAttach[attachCount] = userDataB.Index;
                    attachCount += 1;
                }
            }
            else
            {
                // Only expect evts for the player
                Debug.Assert(bodyIdB == m_playerId);
                var userDataA = (BodyUserData?)Body.GetUserData(bodyIdA);
                if (userDataA == null)
                {
                    if (evt.ShapeIdB != m_coreShapeId && destroyCount < e_count)
                    {
                        // player non-core shape hit the wall

                        bool found = false;
                        for (int j = 0; j < destroyCount; ++j)
                        {
                            if (evt.ShapeIdB == shapesToDestroy[j])
                            {
                                found = true;
                                break;
                            }
                        }

                        // avoid double deletion
                        if (found == false)
                        {
                            shapesToDestroy[destroyCount] = evt.ShapeIdB;
                            destroyCount += 1;
                        }
                    }
                }
                else if (attachCount < e_count)
                {
                    debrisToAttach[attachCount] = userDataA.Index;
                    attachCount += 1;
                }
            }
        }

        // Attach debris to player body
        for (int i = 0; i < attachCount; ++i)
        {
            int index = debrisToAttach[i];
            BodyId debrisId = m_debrisIds[index];
            if (debrisId.IsNotNull)
            {
                continue;
            }

            Transform playerTransform = Body.GetTransform(m_playerId);
            Transform debrisTransform = Body.GetTransform(debrisId);
            Transform relativeTransform = B2Math.InvMulTransforms(playerTransform, debrisTransform);

            int shapeCount = Body.GetShapeCount(debrisId);
            if (shapeCount == 0)
            {
                continue;
            }

            var shapeId = Body.GetShape(debrisId);

            ShapeType type = Shape.GetType(shapeId);

            ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
            shapeDef.EnableContactEvents = true;

            switch (type)
            {
            case ShapeType.CircleShape:
            {
                Circle circle = Shape.GetCircle(shapeId);
                circle.Center = B2Math.TransformPoint(relativeTransform, circle.Center);

                Shape.CreateCircleShape(m_playerId, shapeDef, circle);
            }
                break;

            case ShapeType.CapsuleShape:
            {
                Capsule capsule = Shape.GetCapsule(shapeId);
                capsule.Center1 = B2Math.TransformPoint(relativeTransform, capsule.Center1);
                capsule.Center2 = B2Math.TransformPoint(relativeTransform, capsule.Center2);

                Shape.CreateCapsuleShape(m_playerId, shapeDef, capsule);
            }
                break;

            case ShapeType.PolygonShape:
            {
                Polygon originalPolygon = Shape.GetPolygon(shapeId);
                Polygon polygon = Geometry.TransformPolygon(relativeTransform, originalPolygon);

                Shape.CreatePolygonShape(m_playerId, shapeDef, polygon);
            }
                break;

            default:
                Debug.Assert(false);
                break;
            }

            Body.DestroyBody(debrisId);
            m_debrisIds[index] = BodyId.NullId;
        }

        for (int i = 0; i < destroyCount; ++i)
        {
            Shape.DestroyShape(shapesToDestroy[i]);
        }

        if (Settings.Hertz > 0.0f && Settings.Pause == false)
        {
            m_wait -= 1.0f / Settings.Hertz;
            if (m_wait < 0.0f)
            {
                SpawnDebris();
                m_wait += 0.5f;
            }
        }
    }

    BodyId m_playerId;

    ShapeId m_coreShapeId;

    BodyId[] m_debrisIds = new BodyId[e_count];

    BodyUserData[] m_bodyUserData = new BodyUserData[e_count].Fill();

    protected float m_force;

    float m_wait;
};