using System.Diagnostics;
using Box2DSharp;

namespace ConsoleTest.Base;

public class Human
{
    public class Bone
    {
        public BodyId BodyId;

        public JointId JointId;

        public float FrictionScale;

        public int ParentIndex;
    }

    public struct BoneType
    {
        public const int Hip = 0;

        public const int Torso = 1;

        public const int Head = 2;

        public const int UpperLeftLeg = 3;

        public const int LowerLeftLeg = 4;

        public const int UpperRightLeg = 5;

        public const int LowerRightLeg = 6;

        public const int UpperLeftArm = 7;

        public const int LowerLeftArm = 8;

        public const int UpperRightArm = 9;

        public const int LowerRightArm = 10;
    }

    public const int BoneCount = 11;

    private readonly Bone[] _bones = new Bone[BoneCount];

    public float Scale;

    public bool IsSpawned;

    public Human()
    {
        for (int i = 0; i < BoneCount; ++i)
        {
            _bones[i] = new Bone
            {
                BodyId = BodyId.NullId,
                JointId = JointId.NullId,
                FrictionScale = 1.0f,
                ParentIndex = -1
            };
        }

        Scale = 1.0f;
        IsSpawned = false;
    }

    public void Spawn(WorldId worldId, Vec2 position, float scale, float frictionTorque, float hertz, float dampingRatio, int groupIndex, object? userData, bool colorize)
    {
        Debug.Assert(IsSpawned == false);

        BodyDef bodyDef = BodyDef.DefaultBodyDef();
        bodyDef.Type = BodyType.DynamicBody;
        bodyDef.SleepThreshold = 0.1f;
        bodyDef.UserData = userData;

        ShapeDef shapeDef = ShapeDef.DefaultShapeDef();
        shapeDef.Friction = 0.2f;
        shapeDef.Filter.GroupIndex = -groupIndex;
        shapeDef.Filter.MaskBits = 1;

        ShapeDef footShapeDef = shapeDef;
        footShapeDef.Friction = 0.05f;

        if (colorize)
        {
            footShapeDef.CustomColor = (uint)(uint)B2HexColor.SaddleBrown;
        }

        Scale = scale;
        float s = scale;
        float maxTorque = frictionTorque * s;
        bool enableMotor = true;
        bool enableLimit = true;
        float drawSize = 0.05f;

        B2HexColor shirtColor = B2HexColor.MediumTurquoise;
        B2HexColor pantColor = B2HexColor.DodgerBlue;

        B2HexColor[] skinColors = [B2HexColor.NavajoWhite, B2HexColor.LightYellow, B2HexColor.Peru, B2HexColor.Tan];
        B2HexColor skinColor = skinColors[groupIndex % 4];

        // hip
        {
            Bone bone = _bones[BoneType.Hip];
            bone.ParentIndex = -1;

            bodyDef.Position = (0.0f, 0.95f * s) + position;

            bodyDef.LinearDamping = 0.0f;
            bone.BodyId = Body.CreateBody(worldId, bodyDef);

            if (colorize)
            {
                shapeDef.CustomColor = (uint)pantColor;
            }

            Capsule capsule = ((0.0f, -0.02f * s), (0.0f, 0.02f * s), 0.095f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);
        }

        // torso
        {
            Bone bone = _bones[BoneType.Torso];
            bone.ParentIndex = BoneType.Hip;

            bodyDef.Position = (0.0f, 1.2f * s) + position;
            bodyDef.LinearDamping = 0.0f;

            // bodyDef.type = _staticBody;
            bone.BodyId = Body.CreateBody(worldId, bodyDef);
            bone.FrictionScale = 0.5f;
            bodyDef.Type = BodyType.DynamicBody;

            if (colorize)
            {
                shapeDef.CustomColor = (uint)shirtColor;
            }

            Capsule capsule = ((0.0f, -0.135f * s), (0.0f, 0.135f * s), 0.09f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);

            Vec2 pivot = (0.0f, 1.0f * s) + position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = _bones[bone.ParentIndex].BodyId;
            jointDef.BodyIdB = bone.BodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.EnableLimit = enableLimit;
            jointDef.LowerAngle = -0.25f * B2Math.Pi;
            jointDef.UpperAngle = 0.0f;
            jointDef.EnableMotor = enableMotor;
            jointDef.MaxMotorTorque = bone.FrictionScale * maxTorque;
            jointDef.EnableSpring = hertz > 0.0f;
            jointDef.Hertz = hertz;
            jointDef.DampingRatio = dampingRatio;
            jointDef.DrawSize = drawSize;

            bone.JointId = Joint.CreateRevoluteJoint(worldId, jointDef);
        }

        // head
        {
            Bone bone = _bones[BoneType.Head];
            bone.ParentIndex = BoneType.Torso;

            bodyDef.Position = (0.0f * s, 1.5f * s) + position;
            bodyDef.LinearDamping = 0.1f;

            bone.BodyId = Body.CreateBody(worldId, bodyDef);
            bone.FrictionScale = 0.25f;

            if (colorize)
            {
                shapeDef.CustomColor = (uint)skinColor;
            }

            Capsule capsule = ((0.0f, -0.0325f * s), (0.0f, 0.0325f * s), 0.08f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);

            // neck
            capsule = ((0.0f, -0.12f * s), (0.0f, -0.08f * s), 0.05f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);

            Vec2 pivot = (0.0f, 1.4f * s) + position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = _bones[bone.ParentIndex].BodyId;
            jointDef.BodyIdB = bone.BodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.EnableLimit = enableLimit;
            jointDef.LowerAngle = -0.3f * B2Math.Pi;
            jointDef.UpperAngle = 0.1f * B2Math.Pi;
            jointDef.EnableMotor = enableMotor;
            jointDef.MaxMotorTorque = bone.FrictionScale * maxTorque;
            jointDef.EnableSpring = hertz > 0.0f;
            jointDef.Hertz = hertz;
            jointDef.DampingRatio = dampingRatio;
            jointDef.DrawSize = drawSize;

            bone.JointId = Joint.CreateRevoluteJoint(worldId, jointDef);
        }

        // upper left leg
        {
            Bone bone = _bones[BoneType.UpperLeftLeg];
            bone.ParentIndex = BoneType.Hip;

            bodyDef.Position = (0.0f, 0.775f * s) + position;
            bodyDef.LinearDamping = 0.0f;
            bone.BodyId = Body.CreateBody(worldId, bodyDef);
            bone.FrictionScale = 1.0f;

            if (colorize)
            {
                shapeDef.CustomColor = (uint)pantColor;
            }

            Capsule capsule = ((0.0f, -0.125f * s), (0.0f, 0.125f * s), 0.06f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);

            Vec2 pivot = (0.0f, 0.9f * s) + position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = _bones[bone.ParentIndex].BodyId;
            jointDef.BodyIdB = bone.BodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.EnableLimit = enableLimit;
            jointDef.LowerAngle = -0.05f * B2Math.Pi;
            jointDef.UpperAngle = 0.4f * B2Math.Pi;
            jointDef.EnableMotor = enableMotor;
            jointDef.MaxMotorTorque = bone.FrictionScale * maxTorque;
            jointDef.EnableSpring = hertz > 0.0f;
            jointDef.Hertz = hertz;
            jointDef.DampingRatio = dampingRatio;
            jointDef.DrawSize = drawSize;

            bone.JointId = Joint.CreateRevoluteJoint(worldId, jointDef);
        }

        // lower left leg
        {
            Bone bone = _bones[BoneType.LowerLeftLeg];
            bone.ParentIndex = BoneType.UpperLeftLeg;

            bodyDef.Position = (0.0f, 0.475f * s) + position;
            bodyDef.LinearDamping = 0.0f;
            bone.BodyId = Body.CreateBody(worldId, bodyDef);
            bone.FrictionScale = 0.5f;

            if (colorize)
            {
                shapeDef.CustomColor = (uint)pantColor;
            }

            Capsule capsule = ((0.0f, -0.14f * s), (0.0f, 0.125f * s), 0.05f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);

            // Polygon box = MakeOffsetBox(0.1f * s, 0.03f * s, {0.05f * s, -0.175f * s}, 0.0f);
            // CreatePolygonShape(bone.BodyId, &shapeDef, &box);

            capsule = ((-0.02f * s, -0.175f * s), (0.13f * s, -0.175f * s), 0.03f * s);
            Shape.CreateCapsuleShape(bone.BodyId, footShapeDef, capsule);

            Vec2 pivot = (0.0f, 0.625f * s) + position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = _bones[bone.ParentIndex].BodyId;
            jointDef.BodyIdB = bone.BodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.EnableLimit = enableLimit;
            jointDef.LowerAngle = -0.5f * B2Math.Pi;
            jointDef.UpperAngle = -0.02f * B2Math.Pi;
            jointDef.EnableMotor = enableMotor;
            jointDef.MaxMotorTorque = bone.FrictionScale * maxTorque;
            jointDef.EnableSpring = hertz > 0.0f;
            jointDef.Hertz = hertz;
            jointDef.DampingRatio = dampingRatio;
            jointDef.DrawSize = drawSize;

            bone.JointId = Joint.CreateRevoluteJoint(worldId, jointDef);
        }

        // upper right leg
        {
            Bone bone = _bones[BoneType.UpperRightLeg];
            bone.ParentIndex = BoneType.Hip;

            bodyDef.Position = (0.0f, 0.775f * s) + position;
            bodyDef.LinearDamping = 0.0f;
            bone.BodyId = Body.CreateBody(worldId, bodyDef);
            bone.FrictionScale = 1.0f;

            if (colorize)
            {
                shapeDef.CustomColor = (uint)pantColor;
            }

            Capsule capsule = ((0.0f, -0.125f * s), (0.0f, 0.125f * s), 0.06f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);

            Vec2 pivot = (0.0f, 0.9f * s) + position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = _bones[bone.ParentIndex].BodyId;
            jointDef.BodyIdB = bone.BodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.EnableLimit = enableLimit;
            jointDef.LowerAngle = -0.05f * B2Math.Pi;
            jointDef.UpperAngle = 0.4f * B2Math.Pi;
            jointDef.EnableMotor = enableMotor;
            jointDef.MaxMotorTorque = bone.FrictionScale * maxTorque;
            jointDef.EnableSpring = hertz > 0.0f;
            jointDef.Hertz = hertz;
            jointDef.DampingRatio = dampingRatio;
            jointDef.DrawSize = drawSize;

            bone.JointId = Joint.CreateRevoluteJoint(worldId, jointDef);
        }

        // lower right leg
        {
            Bone bone = _bones[BoneType.LowerRightLeg];
            bone.ParentIndex = BoneType.UpperRightLeg;

            bodyDef.Position = (0.0f, 0.475f * s) + position;
            bodyDef.LinearDamping = 0.0f;
            bone.BodyId = Body.CreateBody(worldId, bodyDef);
            bone.FrictionScale = 0.5f;

            if (colorize)
            {
                shapeDef.CustomColor = (uint)pantColor;
            }

            Capsule capsule = ((0.0f, -0.14f * s), (0.0f, 0.125f * s), 0.05f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);

            // Polygon box = MakeOffsetBox(0.1f * s, 0.03f * s, {0.05f * s, -0.175f * s}, 0.0f);
            // CreatePolygonShape(bone.BodyId, &shapeDef, &box);

            capsule = ((-0.02f * s, -0.175f * s), (0.13f * s, -0.175f * s), 0.03f * s);
            Shape.CreateCapsuleShape(bone.BodyId, footShapeDef, capsule);

            Vec2 pivot = (0.0f, 0.625f * s) + position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = _bones[bone.ParentIndex].BodyId;
            jointDef.BodyIdB = bone.BodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.EnableLimit = enableLimit;
            jointDef.LowerAngle = -0.5f * B2Math.Pi;
            jointDef.UpperAngle = -0.02f * B2Math.Pi;
            jointDef.EnableMotor = enableMotor;
            jointDef.MaxMotorTorque = bone.FrictionScale * maxTorque;
            jointDef.EnableSpring = hertz > 0.0f;
            jointDef.Hertz = hertz;
            jointDef.DampingRatio = dampingRatio;
            jointDef.DrawSize = drawSize;

            bone.JointId = Joint.CreateRevoluteJoint(worldId, jointDef);
        }

        // upper left arm
        {
            Bone bone = _bones[BoneType.UpperLeftArm];
            bone.ParentIndex = BoneType.Torso;
            bone.FrictionScale = 0.5f;

            bodyDef.Position = (0.0f, 1.225f * s) + position;
            bodyDef.LinearDamping = 0.0f;
            bone.BodyId = Body.CreateBody(worldId, bodyDef);

            if (colorize)
            {
                shapeDef.CustomColor = (uint)shirtColor;
            }

            Capsule capsule = ((0.0f, -0.125f * s), (0.0f, 0.125f * s), 0.035f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);

            Vec2 pivot = (0.0f, 1.35f * s) + position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = _bones[bone.ParentIndex].BodyId;
            jointDef.BodyIdB = bone.BodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.EnableLimit = enableLimit;
            jointDef.LowerAngle = -0.1f * B2Math.Pi;
            jointDef.UpperAngle = 0.8f * B2Math.Pi;
            jointDef.EnableMotor = enableMotor;
            jointDef.MaxMotorTorque = bone.FrictionScale * maxTorque;
            jointDef.EnableSpring = hertz > 0.0f;
            jointDef.Hertz = hertz;
            jointDef.DampingRatio = dampingRatio;
            jointDef.DrawSize = drawSize;

            bone.JointId = Joint.CreateRevoluteJoint(worldId, jointDef);
        }

        // lower left arm
        {
            Bone bone = _bones[BoneType.LowerLeftArm];
            bone.ParentIndex = BoneType.UpperLeftArm;

            bodyDef.Position = (0.0f, 0.975f * s) + position;
            bodyDef.LinearDamping = 0.1f;
            bone.BodyId = Body.CreateBody(worldId, bodyDef);
            bone.FrictionScale = 0.1f;

            if (colorize)
            {
                shapeDef.CustomColor = (uint)skinColor;
            }

            Capsule capsule = ((0.0f, -0.125f * s), (0.0f, 0.125f * s), 0.03f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);

            Vec2 pivot = (0.0f, 1.1f * s) + position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = _bones[bone.ParentIndex].BodyId;
            jointDef.BodyIdB = bone.BodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.EnableLimit = enableLimit;
            jointDef.LowerAngle = 0.01f * B2Math.Pi;
            jointDef.UpperAngle = 0.5f * B2Math.Pi;
            jointDef.EnableMotor = enableMotor;
            jointDef.MaxMotorTorque = bone.FrictionScale * maxTorque;
            jointDef.EnableSpring = hertz > 0.0f;
            jointDef.Hertz = hertz;
            jointDef.DampingRatio = dampingRatio;
            jointDef.DrawSize = drawSize;

            bone.JointId = Joint.CreateRevoluteJoint(worldId, jointDef);
        }

        // upper right arm
        {
            Bone bone = _bones[BoneType.UpperRightArm];
            bone.ParentIndex = BoneType.Torso;

            bodyDef.Position = (0.0f, 1.225f * s) + position;
            bodyDef.LinearDamping = 0.0f;
            bone.BodyId = Body.CreateBody(worldId, bodyDef);
            bone.FrictionScale = 0.5f;

            if (colorize)
            {
                shapeDef.CustomColor = (uint)shirtColor;
            }

            Capsule capsule = ((0.0f, -0.125f * s), (0.0f, 0.125f * s), 0.035f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);

            Vec2 pivot = (0.0f, 1.35f * s) + position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = _bones[bone.ParentIndex].BodyId;
            jointDef.BodyIdB = bone.BodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.EnableLimit = enableLimit;
            jointDef.LowerAngle = -0.1f * B2Math.Pi;
            jointDef.UpperAngle = 0.8f * B2Math.Pi;
            jointDef.EnableMotor = enableMotor;
            jointDef.MaxMotorTorque = bone.FrictionScale * maxTorque;
            jointDef.EnableSpring = hertz > 0.0f;
            jointDef.Hertz = hertz;
            jointDef.DampingRatio = dampingRatio;
            jointDef.DrawSize = drawSize;

            bone.JointId = Joint.CreateRevoluteJoint(worldId, jointDef);
        }

        // lower right arm
        {
            Bone bone = _bones[BoneType.LowerRightArm];
            bone.ParentIndex = BoneType.UpperRightArm;

            bodyDef.Position = (0.0f, 0.975f * s) + position;
            bodyDef.LinearDamping = 0.1f;
            bone.BodyId = Body.CreateBody(worldId, bodyDef);
            bone.FrictionScale = 0.1f;

            if (colorize)
            {
                shapeDef.CustomColor = (uint)skinColor;
            }

            Capsule capsule = ((0.0f, -0.125f * s), (0.0f, 0.125f * s), 0.03f * s);
            Shape.CreateCapsuleShape(bone.BodyId, shapeDef, capsule);

            Vec2 pivot = (0.0f, 1.1f * s) + position;
            RevoluteJointDef jointDef = RevoluteJointDef.DefaultRevoluteJointDef();
            jointDef.BodyIdA = _bones[bone.ParentIndex].BodyId;
            jointDef.BodyIdB = bone.BodyId;
            jointDef.LocalAnchorA = Body.GetLocalPoint(jointDef.BodyIdA, pivot);
            jointDef.LocalAnchorB = Body.GetLocalPoint(jointDef.BodyIdB, pivot);
            jointDef.EnableLimit = enableLimit;
            jointDef.LowerAngle = 0.01f * B2Math.Pi;
            jointDef.UpperAngle = 0.5f * B2Math.Pi;
            jointDef.EnableMotor = enableMotor;
            jointDef.MaxMotorTorque = bone.FrictionScale * maxTorque;
            jointDef.EnableSpring = hertz > 0.0f;
            jointDef.Hertz = hertz;
            jointDef.DampingRatio = dampingRatio;
            jointDef.DrawSize = drawSize;

            bone.JointId = Joint.CreateRevoluteJoint(worldId, jointDef);
        }

        IsSpawned = true;
    }

    public void Despawn()
    {
        Debug.Assert(IsSpawned == true);

        for (var i = 0; i < BoneCount; ++i)
        {
            if (_bones[i].JointId.IsNull)
            {
                continue;
            }

            Joint.DestroyJoint(_bones[i].JointId);
            _bones[i].JointId = JointId.NullId;
        }

        for (var i = 0; i < BoneCount; ++i)
        {
            if (_bones[i].BodyId.IsNull)
            {
                continue;
            }

            Body.DestroyBody(_bones[i].BodyId);
            _bones[i].BodyId = BodyId.NullId;
        }

        IsSpawned = false;
    }
}