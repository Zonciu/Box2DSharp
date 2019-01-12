using System.Collections.Generic;
using System.Linq;
using Box2DSharp.Collision;
using Box2DSharp.Dynamics.Contacts;

namespace Box2DSharp.Dynamics
{
    // Delegate of b2World.
    public class ContactManager
    {
        public static readonly IContactFilter DefaultContactFilter = new DefaultContactFilter();

        public static readonly IContactListener DefaultContactListener = new NullContactListener();

        public readonly BroadPhase BroadPhase = new BroadPhase();

        public IContactFilter ContactFilter = DefaultContactFilter;

        public readonly LinkedList<Contact> ContactList = new LinkedList<Contact>();

        public IContactListener ContactListener = DefaultContactListener;

        // Broad-phase callback.
        public void AddPair(FixtureProxy proxyA, FixtureProxy proxyB)
        {
            var fixtureA = proxyA.Fixture;
            var fixtureB = proxyB.Fixture;

            var indexA = proxyA.ChildIndex;
            var indexB = proxyB.ChildIndex;

            var bodyA = fixtureA.Body;
            var bodyB = fixtureB.Body;

            // Are the fixtures on the same body?
            if (bodyA == bodyB)
            {
                return;
            }

            // TODO_ERIN use a hash table to remove a potential bottleneck when both
            // bodies have a lot of contacts.
            // Does a contact already exist?
            var node1 = bodyB.ContactEdges.First;
            while (node1 != null)
            {
                var contactEdge = node1.Value;
                node1 = node1.Next;

                var fA = contactEdge.Contact.GetFixtureA();
                var fB = contactEdge.Contact.GetFixtureB();
                var iA = contactEdge.Contact.GetChildIndexA();
                var iB = contactEdge.Contact.GetChildIndexB();

                if (fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB)
                {
                    // A contact already exists.
                    return;
                }

                if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA)
                {
                    // A contact already exists.
                    return;
                }
            }

            // Does a joint override collision? Is at least one body dynamic?
            if (bodyB.ShouldCollide(bodyA) == false)
            {
                return;
            }

            // Check user filtering.
            if (ContactFilter?.ShouldCollide(fixtureA, fixtureB) == false)
            {
                return;
            }

            // Call the factory.
            var c = Contact.CreateContact(
                fixtureA,
                indexA,
                fixtureB,
                indexB);
            if (c == default)
            {
                return;
            }

            // Contact creation may swap fixtures.
            fixtureA = c.GetFixtureA();
            fixtureB = c.GetFixtureB();
            indexA = c.GetChildIndexA();
            indexB = c.GetChildIndexB();
            bodyA = fixtureA.Body;
            bodyB = fixtureB.Body;

            // Insert into the world.
            c.Node = ContactList.AddFirst(c);

            // Connect to island graph.

            // Connect to body A
            c.NodeA.Contact = c;
            c.NodeA.Other = bodyB;
            c.NodeA.Node = bodyA.ContactEdges.AddFirst(c.NodeA);

            // Connect to body B
            c.NodeB.Contact = c;
            c.NodeB.Other = bodyA;
            c.NodeB.Node = bodyB.ContactEdges.AddFirst(c.NodeB);

            // Wake up the bodies
            if (fixtureA.IsSensor == false && fixtureB.IsSensor == false)
            {
                bodyA.IsAwake = true;
                bodyB.IsAwake = true;
            }
        }

        public void FindNewContacts()
        {
            BroadPhase.UpdatePairs(AddPair);
        }

        public void Destroy(Contact c)
        {
            var fixtureA = c.GetFixtureA();
            var fixtureB = c.GetFixtureB();
            var bodyA = fixtureA.Body;
            var bodyB = fixtureB.Body;

            if (c.IsTouching()) // 存在接触监听器且当前接触点接触,则触发结束接触
            {
                ContactListener?.EndContact(c);
            }

            // Remove from the world.
            ContactList.Remove(c.Node);

            // Remove from body 1
            bodyA.ContactEdges.Remove(c.NodeA.Node);

            // Remove from body 2
            bodyB.ContactEdges.Remove(c.NodeB.Node);

            // Call the factory.
            Contact.DestroyContact(c);
        }

        public void Collide()
        {
            var current = ContactList.First;

            // Update awake contacts.
            while (current != default)
            {
                var c = current.Value;

                var fixtureA = c.GetFixtureA();
                var fixtureB = c.GetFixtureB();
                var indexA = c.GetChildIndexA();
                var indexB = c.GetChildIndexB();
                var bodyA = fixtureA.Body;
                var bodyB = fixtureB.Body;

                // Is this contact flagged for filtering?
                if (c.HasFlag(Contact.ContactFlag.FilterFlag))
                {
                    // Should these bodies collide?
                    if (!bodyB.ShouldCollide(bodyA))
                    {
                        current = current.Next;
                        Destroy(c);
                        continue;
                    }

                    // Check user filtering.
                    if (ContactFilter?.ShouldCollide(fixtureA, fixtureB) == false)
                    {
                        current = current.Next;
                        Destroy(c);
                        continue;
                    }

                    // Clear the filtering flag.
                    c.Flags &= ~Contact.ContactFlag.FilterFlag;
                }

                var activeA = bodyA.IsAwake && bodyA.BodyType != BodyType.StaticBody;
                var activeB = bodyB.IsAwake && bodyB.BodyType != BodyType.StaticBody;

                // At least one body must be awake and it must be dynamic or kinematic.
                if (activeA == false && activeB == false)
                {
                    current = current.Next;
                    continue;
                }

                var proxyIdA = fixtureA.Proxies[indexA].ProxyId;
                var proxyIdB = fixtureB.Proxies[indexB].ProxyId;
                var overlap = BroadPhase.TestOverlap(proxyIdA, proxyIdB);

                // Here we destroy contacts that cease to overlap in the broad-phase.
                if (overlap == false)
                {
                    current = current.Next;
                    Destroy(c);
                    continue;
                }

                // The contact persists.
                c.Update(ContactListener);
                current = current.Next;
            }
        }
    }
}