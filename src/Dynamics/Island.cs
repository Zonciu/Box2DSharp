using System;
using System.Diagnostics;
using System.Numerics;
using Box2DSharp.Common;
using Box2DSharp.Dynamics.Contacts;
using Box2DSharp.Dynamics.Joints;
using Box2DSharp.Dynamics.Listeners;

namespace Box2DSharp.Dynamics
{
    /// This is an internal class.
    public class Island
    {
        internal Body[] Bodies;

        internal int BodyCount;

        internal Contact[] Contacts;

        internal int ContactCount;

        internal Joint[] Joints;

        internal int JointCount;

        internal Position[] Positions;

        internal Velocity[] Velocities;

        internal IContactListener ContactListener;

        public Island(
            int              bodyCapacity,
            int              contactCapacity,
            int              jointCapacity,
            IContactListener contactListener)
        {
            BodyCount    = 0;
            ContactCount = 0;
            JointCount   = 0;

            ContactListener = contactListener;

            Bodies   = new Body[bodyCapacity];
            Contacts = new Contact[contactCapacity];
            Joints   = new Joint[jointCapacity];

            Velocities = new Velocity[bodyCapacity];
            Positions  = new Position[bodyCapacity];
        }

        internal void Clear()
        {
            BodyCount    = 0;
            ContactCount = 0;
            JointCount   = 0;
        }

        internal Profile Solve(in TimeStep step, in Vector2 gravity, bool allowSleep)
        {
            var profile = new Profile();
            var timer   = Stopwatch.StartNew();

            var h = step.dt;

            // Integrate velocities and apply damping. Initialize the body state.
            for (var i = 0; i < BodyCount; ++i)
            {
                var b = Bodies[i];

                var c = b._sweep.c;
                var a = b._sweep.a;
                var v = b._linearVelocity;
                var w = b._angularVelocity;

                // Store positions for continuous collision.
                b._sweep.c0 = b._sweep.c;
                b._sweep.a0 = b._sweep.a;

                if (b._type == BodyType.DynamicBody)
                {
                    // Integrate velocities.
                    v += h * (b._gravityScale * gravity + b._invMass * b._force);
                    w += h * b._inverseInertia * b._torque;

                    // Apply damping.
                    // ODE: dv/dt + c * v = 0
                    // Solution: v(t) = v0 * exp(-c * t)
                    // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                    // v2 = exp(-c * dt) * v1
                    // Pade approximation:
                    // v2 = v1 * 1 / (1 + c * dt)
                    v *= 1.0f / (1.0f + h * b._linearDamping);
                    w *= 1.0f / (1.0f + h * b._angularDamping);
                }

                Positions[i].Center = c;
                Positions[i].Angle  = a;
                Velocities[i].v     = v;
                Velocities[i].w     = w;
            }

            timer.Reset();

            // Solver data
            var solverData = new SolverData
            {
                Step       = step,
                Positions  = Positions,
                Velocities = Velocities
            };

            // Initialize velocity constraints.
            var contactSolverDef = new ContactSolverDef
            {
                step       = step,
                contacts   = Contacts,
                count      = ContactCount,
                positions  = Positions,
                velocities = Velocities
            };

            var contactSolver = new ContactSolver(contactSolverDef);
            contactSolver.InitializeVelocityConstraints();

            if (step.warmStarting)
            {
                contactSolver.WarmStart();
            }

            for (var i = 0; i < JointCount; ++i)
            {
                Joints[i].InitVelocityConstraints(solverData);
            }

            profile.solveInit = timer.ElapsedMilliseconds;

            // Solve velocity constraints
            timer.Reset();
            for (var i = 0; i < step.velocityIterations; ++i)
            {
                for (var j = 0; j < JointCount; ++j)
                {
                    Joints[j].SolveVelocityConstraints(solverData);
                }

                contactSolver.SolveVelocityConstraints();
            }

            // Store impulses for warm starting
            contactSolver.StoreImpulses();
            profile.solveVelocity = timer.ElapsedMilliseconds;

            // Integrate positions
            for (var i = 0; i < BodyCount; ++i)
            {
                var c = Positions[i].Center;
                var a = Positions[i].Angle;
                var v = Velocities[i].v;
                var w = Velocities[i].w;

                // Check for large velocities
                var translation = h * v;
                if (MathUtils.Dot(translation, translation) > Settings.MaxTranslationSquared)
                {
                    var ratio = Settings.MaxTranslation / translation.Length();
                    v *= ratio;
                }

                var rotation = h * w;
                if (rotation * rotation > Settings.MaxRotationSquared)
                {
                    var ratio = Settings.MaxRotation / Math.Abs(rotation);
                    w *= ratio;
                }

                // Integrate
                c += h * v;
                a += h * w;

                Positions[i].Center = c;
                Positions[i].Angle  = a;
                Velocities[i].v     = v;
                Velocities[i].w     = w;
            }

            // Solve position constraints
            timer.Reset();
            var positionSolved = false;
            for (var i = 0; i < step.positionIterations; ++i)
            {
                var contactsOkay = contactSolver.SolvePositionConstraints();

                var jointsOkay = true;
                for (var j = 0; j < JointCount; ++j)
                {
                    var jointOkay = Joints[j].SolvePositionConstraints(solverData);
                    jointsOkay = jointsOkay && jointOkay;
                }

                if (contactsOkay && jointsOkay)
                {
                    // Exit early if the position errors are small.
                    positionSolved = true;
                    break;
                }
            }

            // Copy state buffers back to the bodies
            for (var i = 0; i < BodyCount; ++i)
            {
                var body = Bodies[i];
                body._sweep.c         = Positions[i].Center;
                body._sweep.a         = Positions[i].Angle;
                body._linearVelocity  = Velocities[i].v;
                body._angularVelocity = Velocities[i].w;
                body.SynchronizeTransform();
            }

            profile.solvePosition = timer.ElapsedMilliseconds;

            Report(contactSolver.VelocityConstraints);

            if (allowSleep)
            {
                var minSleepTime = Settings.MaxFloat;

                const float linTolSqr = Settings.LinearSleepTolerance * Settings.LinearSleepTolerance;
                const float angTolSqr = Settings.AngularSleepTolerance * Settings.AngularSleepTolerance;

                for (var i = 0; i < BodyCount; ++i)
                {
                    var b = Bodies[i];
                    if (b.BodyType == BodyType.StaticBody)
                    {
                        continue;
                    }

                    if (!b.HasFlag(BodyFlags.AutoSleep)
                     || b._angularVelocity * b._angularVelocity > angTolSqr
                     || MathUtils.Dot(b._linearVelocity, b._linearVelocity) > linTolSqr)
                    {
                        b._sleepTime = 0.0f;
                        minSleepTime = 0.0f;
                    }
                    else
                    {
                        b._sleepTime += h;
                        minSleepTime =  Math.Min(minSleepTime, b._sleepTime);
                    }
                }

                if (minSleepTime >= Settings.TimeToSleep && positionSolved)
                {
                    for (var i = 0; i < BodyCount; ++i)
                    {
                        var b = Bodies[i];
                        b.IsAwake = false;
                    }
                }
            }

            return profile;
        }

        internal void SolveTOI(in TimeStep subStep, int toiIndexA, int toiIndexB)
        {
            Debug.Assert(toiIndexA < BodyCount);
            Debug.Assert(toiIndexB < BodyCount);

            // Initialize the body state.
            for (var i = 0; i < BodyCount; ++i)
            {
                var b = Bodies[i];
                Positions[i].Center = b._sweep.c;
                Positions[i].Angle  = b._sweep.a;
                Velocities[i].v     = b._linearVelocity;
                Velocities[i].w     = b._angularVelocity;
            }

            var contactSolverDef = new ContactSolverDef
            {
                contacts   = Contacts,
                count      = ContactCount,
                step       = subStep,
                positions  = Positions,
                velocities = Velocities
            };
            var contactSolver = new ContactSolver(contactSolverDef);

            // Solve position constraints.
            for (var i = 0; i < subStep.positionIterations; ++i)
            {
                var contactsOkay = contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
                if (contactsOkay)
                {
                    break;
                }
            }

#if FALSE

// Is the new position really safe?
            for (int i = 0; i < m_contactCount; ++i)
            {
                var c = m_contacts[i];
                var fA = c.GetFixtureA();
                var fB = c.GetFixtureB();

                var bA = fA.GetBody();
                var bB = fB.GetBody();

                int indexA = c.GetChildIndexA();
                int indexB = c.GetChildIndexB();

                b2DistanceInput input = new b2DistanceInput();
                input.proxyA.Set(fA.GetShape(), indexA);
                input.proxyB.Set(fB.GetShape(), indexB);
                input.transformA = bA.GetTransform();
                input.transformB = bB.GetTransform();
                input.useRadii = false;

                b2DistanceOutput output = new b2DistanceOutput();
                SimplexCache     cache = new SimplexCache {count = 0};
                DistanceAlgorithm.b2Distance(ref output, ref cache, ref input);

                if (output.distance.Equals(0) || cache.count == 3)
                {
                    cache.count += 0;
                }
            }
#endif

            // Leap of faith to new safe state.
            Bodies[toiIndexA]._sweep.c0 = Positions[toiIndexA].Center;
            Bodies[toiIndexA]._sweep.a0 = Positions[toiIndexA].Angle;
            Bodies[toiIndexB]._sweep.c0 = Positions[toiIndexB].Center;
            Bodies[toiIndexB]._sweep.a0 = Positions[toiIndexB].Angle;

            // No warm starting is needed for TOI events because warm
            // starting impulses were applied in the discrete solver.
            contactSolver.InitializeVelocityConstraints();

            // Solve velocity constraints.
            for (var i = 0; i < subStep.velocityIterations; ++i)
            {
                contactSolver.SolveVelocityConstraints();
            }

            // Don't store the TOI contact forces for warm starting
            // because they can be quite large.

            var h = subStep.dt;

            // Integrate positions
            for (var i = 0; i < BodyCount; ++i)
            {
                var c = Positions[i].Center;
                var a = Positions[i].Angle;
                var v = Velocities[i].v;
                var w = Velocities[i].w;

                // Check for large velocities
                var translation = h * v;
                if (MathUtils.Dot(translation, translation) > Settings.MaxTranslationSquared)
                {
                    var ratio = Settings.MaxTranslation / translation.Length();
                    v *= ratio;
                }

                var rotation = h * w;
                if (rotation * rotation > Settings.MaxRotationSquared)
                {
                    var ratio = Settings.MaxRotation / Math.Abs(rotation);
                    w *= ratio;
                }

                // Integrate
                c += h * v;
                a += h * w;

                Positions[i].Center = c;
                Positions[i].Angle  = a;
                Velocities[i].v     = v;
                Velocities[i].w     = w;

                // Sync bodies
                var body = Bodies[i];
                body._sweep.c         = c;
                body._sweep.a         = a;
                body._linearVelocity  = v;
                body._angularVelocity = w;
                body.SynchronizeTransform();
            }

            Report(contactSolver.VelocityConstraints);
        }

        internal void Add(Body body)
        {
            Debug.Assert(BodyCount < Bodies.Length);
            body._islandIndex = BodyCount;
            Bodies[BodyCount] = body;
            ++BodyCount;
        }

        internal void Add(Contact contact)
        {
            Debug.Assert(ContactCount < Contacts.Length);
            Contacts[ContactCount++] = contact;
        }

        internal void Add(Joint joint)
        {
            Debug.Assert(JointCount < Joints.Length);
            Joints[JointCount++] = joint;
        }

        internal void Report(ContactVelocityConstraint[] constraints)
        {
            if (ContactListener == null)
            {
                return;
            }

            for (var i = 0; i < ContactCount; ++i)
            {
                var c = Contacts[i];

                var vc = constraints[i];

                var impulse = ContactImpulse.Create();
                impulse.count = vc.pointCount;
                for (var j = 0; j < vc.pointCount; ++j)
                {
                    impulse.normalImpulses[j]  = vc.points[j].normalImpulse;
                    impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
                }

                ContactListener.PostSolve(c, impulse);
            }
        }
    }
}