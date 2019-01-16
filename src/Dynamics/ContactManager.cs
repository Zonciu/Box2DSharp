using System.Collections.Generic;
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
        public void AddPair(object proxyUserDataA, object proxyUserDataB)
        {
            var proxyA = (FixtureProxy) proxyUserDataA;
            var proxyB = (FixtureProxy) proxyUserDataB;
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

                if ((contactEdge.Contact.FixtureA == fixtureA
                  && contactEdge.Contact.FixtureB == fixtureB
                  && contactEdge.Contact.ChildIndexA == indexA
                  && contactEdge.Contact.ChildIndexB == indexB)
                 || (contactEdge.Contact.FixtureA == fixtureB
                  && contactEdge.Contact.FixtureB == fixtureA
                  && contactEdge.Contact.ChildIndexA == indexB
                  && contactEdge.Contact.ChildIndexB == indexA))
                {
                    // A contact already exists.
                    return;
                }
            }

            if (bodyB.ShouldCollide(bodyA) == false                        // Does a joint override collision? Is at least one body dynamic?
             || ContactFilter?.ShouldCollide(fixtureA, fixtureB) == false) // Check user filtering.
            {
                return;
            }

            // Call the factory.
            var c = Contact.CreateContact(fixtureA, indexA, fixtureB, indexB);
            if (c == default)
            {
                return;
            }

            // Contact creation may swap fixtures.
            fixtureA = c.FixtureA;
            fixtureB = c.FixtureB;
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
            var fixtureA = c.FixtureA;
            var fixtureB = c.FixtureB;
            var bodyA = fixtureA.Body;
            var bodyB = fixtureB.Body;

            if (c.IsTouching) // 存在接触监听器且当前接触点接触,则触发结束接触
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
            var node = ContactList.First;

            // Update awake contacts.
            while (node != default)
            {
                var c = node.Value;
                node = node.Next;
                var fixtureA = c.FixtureA;
                var fixtureB = c.FixtureB;
                var indexA = c.ChildIndexA;
                var indexB = c.ChildIndexB;
                var bodyA = fixtureA.Body;
                var bodyB = fixtureB.Body;

                // Is this contact flagged for filtering?
                if (c.HasFlag(Contact.ContactFlag.FilterFlag))
                {
                    // Should these bodies collide?
                    if (bodyB.ShouldCollide(bodyA) == false)
                    {
                        Destroy(c);
                        continue;
                    }

                    // Check user filtering.
                    if (ContactFilter?.ShouldCollide(fixtureA, fixtureB) == false)
                    {
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
                    continue;
                }

                var proxyIdA = fixtureA.Proxies[indexA].ProxyId;
                var proxyIdB = fixtureB.Proxies[indexB].ProxyId;
                var overlap = BroadPhase.TestOverlap(proxyIdA, proxyIdB);

                // Here we destroy contacts that cease to overlap in the broad-phase.
                if (overlap == false)
                {
                    Destroy(c);
                    continue;
                }

                // The contact persists.
                c.Update(ContactListener);
            }
        }
    }
}