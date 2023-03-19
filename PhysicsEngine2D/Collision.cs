using Raylib_cs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace PhysicsEngine2D
{   
    public readonly struct Manifold
    {
        public readonly Rigidbody a;
        public readonly Rigidbody b;
        public readonly Vec2 normal;
        public readonly float penDepth;
        public readonly List<Vec2> contacts;

        public Manifold(Rigidbody a, Rigidbody b, Vec2 normal, List<Vec2> contacts, float penDepth)
        {
            this.a = a;
            this.b = b; 
            this.normal = normal;
            this.contacts = contacts;
            this.penDepth = penDepth;
        }
    }
    public readonly struct AABB
    {
        public readonly Vec2 min;
        public readonly Vec2 max;

        public AABB(Vec2 min, Vec2 max)
        {
            this.min = min;
            this.max = max;
        }
    }
    public readonly struct Pair
    {
        public readonly Rigidbody rbA;
        public readonly Rigidbody rbB;

        public Pair(Rigidbody rbA, Rigidbody rbB)
        {
            this.rbA = rbA;
            this.rbB = rbB;
        }
    }

    public static class Collision
    {
        private static Vec2[] impulseList = new Vec2[2];
        private static Vec2[] raList = new Vec2[2];
        private readonly static Vec2[] rbList = new Vec2[2];
        private static Vec2[] fritionImpulseList = new Vec2[2];
        private static float[] jList = new float[2];

        public static void PolygonVsPolygon(Polygon a, Polygon b)
        {
            Vec2 normal = Vec2.zero;
            float depth = float.MaxValue;

            for (int i = 0; i < a.vertices.Length; i++)
            {
                Vec2 va = a.vertices[i];
                Vec2 vb = a.vertices[(i + 1) % a.vertices.Length];

                Vec2 edge = vb - va;
                Vec2 axis = new Vec2(-edge.y, edge.x).Normalized();

                Mathf.ProjectVertices(a.vertices.ToList(), axis, out float minA, out float maxA);
                Mathf.ProjectVertices(b.vertices.ToList(), axis, out float minB, out float maxB);

                if (minA >= maxB || minB >= maxA) return;

                float axisDepth = Math.Min(maxB - minA, maxA - minB);
                if (axisDepth < depth) { depth = axisDepth; normal = axis; }
            }

            for (int i = 0; i < b.vertices.Length; i++)
            {
                Vec2 va = b.vertices[i];
                Vec2 vb = b.vertices[(i + 1) % b.vertices.Length];

                Vec2 edge = vb - va;
                Vec2 axis = new Vec2(-edge.y, edge.x).Normalized();

                Mathf.ProjectVertices(a.vertices.ToList(), axis, out float minA, out float maxA);
                Mathf.ProjectVertices(b.vertices.ToList(), axis, out float minB, out float maxB);

                if (minA >= maxB || minB >= maxA) return;

                float axisDepth = Math.Min(maxB - minA, maxA - minB);
                if (axisDepth < depth) { depth = axisDepth; normal = axis; }
            }

            Vec2 direction = b.position - a.position;

            if (Vec2.Dot(direction, normal) < 0f) normal = -normal;

            // contact points calculation

            Vec2 c1 = Vec2.zero;
            Vec2 c2 = Vec2.zero;
            float minDistSq = float.MaxValue;

            for (int i = 0; i < a.vertices.Length; i++)
            {
                Vec2 p = a.vertices[i];

                for (int j = 0; j < b.vertices.Length; j++)
                {
                    Vec2 va = b.vertices[j];
                    Vec2 vb = b.vertices[(j + 1) % b.vertices.Length];

                    Vec2.PointToSegmentDistance(p, va, vb, out float distSq, out Vec2 cp);

                    if (Mathf.NearlyEqual(distSq, minDistSq))
                    {   
                        if (!Vec2.NearlyEqual(cp, c1) && !Vec2.NearlyEqual(cp, c2))
                        {
                            c2 = cp;
                        }
                        
                    }
                    else if (distSq < minDistSq)
                    {
                        minDistSq = distSq;
                        c1 = cp;
                    }
                }
            }

            for (int i = 0; i < b.vertices.Length; i++)
            {
                Vec2 p = b.vertices[i];

                for (int j = 0; j < a.vertices.Length; j++)
                {
                    Vec2 va = a.vertices[j];
                    Vec2 vb = a.vertices[(j + 1) % a.vertices.Length];

                    Vec2.PointToSegmentDistance(p, va, vb, out float distSq, out Vec2 cp);

                    if (Mathf.NearlyEqual(distSq, minDistSq))
                    {
                        if (!Vec2.NearlyEqual(cp, c1) && !Vec2.NearlyEqual(cp, c2))
                        {
                            c2 = cp;
                        }

                    }
                    else if (distSq < minDistSq)
                    {
                        minDistSq = distSq;
                        c1 = cp;
                    }
                }
            }

            List<Vec2> cpS = new List<Vec2>();
            if (!Vec2.NearlyEqual(c1, Vec2.zero)) cpS.Add(c1);
            if (!Vec2.NearlyEqual(c2, Vec2.zero)) cpS.Add(c2);

            ResolveCollision(new Manifold(a, b, normal, cpS, depth));
        }

        public static void CircleVsCircle(Circle a, Circle b)
        {
            float d = Vec2.Distance(a.position, b.position);
            if (d < a.radius + b.radius)
            {
                float depth = (a.radius + b.radius) - d;
                Vec2 normal = Vec2.Normalize(new Vec2(a.position.x - b.position.x, a.position.y - b.position.y));

                // contact points calculation

                Vec2 ab = b.position - a.position;
                Vec2 dir = Vec2.Normalize(ab);
                Vec2 cp = a.position + dir * a.radius;
                List<Vec2> cpS = new List<Vec2> { cp };

                ResolveCollision(new Manifold(b, a, normal, cpS, depth));
            }
        }

        public static void CircleVsPolygon(Circle circle, Polygon polygon)
        {
            Vec2 normal = Vec2.zero;
            float depth = float.MaxValue;

            Vec2 axis = Vec2.zero;
            float axisDepth;
            float minA, maxA, minB, maxB;

            for (int i = 0; i < polygon.vertices.Length; i++)
            {
                Vec2 va = polygon.vertices[i];
                Vec2 vb = polygon.vertices[(i + 1) % polygon.vertices.Length];

                Vec2 edge = vb - va;
                axis = new Vec2(-edge.y, edge.x).Normalized();

                Mathf.ProjectVertices(polygon.vertices.ToList(), axis, out minA, out maxA);
                Mathf.ProjectCircle(circle, axis, out minB, out maxB);

                if (minA >= maxB || minB >= maxA) return;

                axisDepth = Math.Min(maxB - minA, maxA - minB);
                if (axisDepth < depth) { depth = axisDepth; normal = axis; }
            }

            Vec2 cp = Vec2.FindClosestPoint(circle.position, polygon.vertices.ToList());
            axis = (cp - circle.position).Normalized();

            Mathf.ProjectVertices(polygon.vertices.ToList(), axis, out minA, out maxA);
            Mathf.ProjectCircle(circle, axis, out minB, out maxB);

            if (minA >= maxB || minB >= maxA) return;

            axisDepth = Math.Min(maxB - minA, maxA - minB);
            if (axisDepth < depth) { depth = axisDepth; normal = axis; }

            Vec2 direction = polygon.position - circle.position;

            if (Vec2.Dot(direction, normal) < 0f) normal = -normal;

            // contact points calculation

            float minDistSq = float.MaxValue;
            Vec2 contactPoint = Vec2.zero;

            for (int i = 0; i < polygon.vertices.Length; i++)
            {
                Vec2 va = polygon.vertices[i];
                Vec2 vb = polygon.vertices[(i + 1) % polygon.vertices.Length];

                Vec2.PointToSegmentDistance(circle.position, va, vb, out float distSq, out Vec2 contact);
                if (distSq < minDistSq) { minDistSq = distSq; contactPoint = contact; }
            }

            List<Vec2> cpS = new List<Vec2> { contactPoint };

            ResolveCollision(new Manifold(circle, polygon, normal, cpS, depth));
        }

        public static bool AabbVsAabb(AABB a, AABB b)
        {
            if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
            if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
            return true;
        }

        public static void ResolveCollision(Manifold manifold)
        {
            Rigidbody a = manifold.a;
            Rigidbody b = manifold.b;
            Vec2 normal = manifold.normal;
            List<Vec2> contacts = manifold.contacts;

            for (int i = 0; i < manifold.contacts.Count; i++)
            {
                Scene.contacts.Add(manifold.contacts[i]);
            }

            float e = Math.Min(a.restitution, b.restitution);
            float sf = (a.staticFriction + b.staticFriction) / 2;
            float df = (a.dynamicFriction + b.dynamicFriction) / 2;

            for (int i = 0; i < contacts.Count; i++)
            {
                impulseList[i] = Vec2.zero;
                raList[i] = Vec2.zero;
                rbList[i] = Vec2.zero;
                fritionImpulseList[i] = Vec2.zero;
                jList[i] = 0f;
            }

            for (int i = 0; i < contacts.Count; i++)
            {
                Vec2 ra = contacts[i] - a.position;
                Vec2 rb = contacts[i] - b.position;

                raList[i] = ra;
                rbList[i] = rb;

                Vec2 raPerp = new Vec2(-ra.y, ra.x);
                Vec2 rbPerp = new Vec2(-rb.y, rb.x);

                Vec2 angularLinearVelocityA = raPerp * a.angularVelocity;
                Vec2 angularLinearVelocityB = rbPerp * b.angularVelocity;

                Vec2 relativeVelocity = (b.velocity + angularLinearVelocityB) - (a.velocity + angularLinearVelocityA);

                float contactVelocityMag = Vec2.Dot(relativeVelocity, normal);

                if (contactVelocityMag > 0.0f) continue;

                float raPerpDotN = Vec2.Dot(raPerp, normal);
                float rbPerpDotN = Vec2.Dot(rbPerp, normal);

                float denom = a.InversedMass + b.InversedMass + (raPerpDotN * raPerpDotN)
                    * a.InversedInertia + (rbPerpDotN * rbPerpDotN) * b.InversedInertia;

                float j = -(1f + e) * contactVelocityMag;
                j /= denom;
                j /= (float)contacts.Count;

                jList[i] = j;

                Vec2 impulse = j * normal;
                impulseList[i] = impulse;
            }

            for (int i = 0; i < contacts.Count; i++)
            {
                Vec2 impulse = impulseList[i];
                Vec2 ra = raList[i];
                Vec2 rb = rbList[i];

                a.AddImpulse(-impulse);
                a.Rotate(-Vec2.Cross(ra, impulse));

                b.AddImpulse(impulse);
                b.Rotate(Vec2.Cross(rb, impulse));              
            }

            PositionCorrection(manifold);

            for (int i = 0; i < contacts.Count; i++)
            {
                Vec2 ra = contacts[i] - a.position;
                Vec2 rb = contacts[i] - b.position;

                raList[i] = ra;
                rbList[i] = rb;

                Vec2 raPerp = new Vec2(-ra.y, ra.x);
                Vec2 rbPerp = new Vec2(-rb.y, rb.x);

                Vec2 angularLinearVelocityA = raPerp * a.angularVelocity;
                Vec2 angularLinearVelocityB = rbPerp * b.angularVelocity;

                Vec2 relativeVelocity = (b.velocity + angularLinearVelocityB) - (a.velocity + angularLinearVelocityA);

                Vec2 tangent = relativeVelocity - Vec2.Dot(relativeVelocity, normal) * normal;
                
                if (Vec2.NearlyEqual(tangent, Vec2.zero))
                {
                    continue;
                }
                else tangent = Vec2.Normalize(tangent);

                float raPerpDotT = Vec2.Dot(raPerp, tangent);
                float rbPerpDotT = Vec2.Dot(rbPerp, tangent);

                float denom = a.InversedMass + b.InversedMass + (raPerpDotT * raPerpDotT)
                    * a.InversedInertia + (rbPerpDotT * rbPerpDotT) * b.InversedInertia;

                float jt = -Vec2.Dot(relativeVelocity, tangent); ;
                jt /= denom;
                jt /= (float)contacts.Count;

                Vec2 frictionImpulse;

                float j = jList[i];

                if (Math.Abs(jt) <= j * sf)
                {
                    frictionImpulse = jt * tangent;
                }
                else
                {
                    frictionImpulse = -j * tangent * df;
                }

                fritionImpulseList[i] = frictionImpulse;
            }

            for (int i = 0; i < contacts.Count; i++)
            {
                Vec2 frictionImpulse = fritionImpulseList[i];
                Vec2 ra = raList[i];
                Vec2 rb = rbList[i];

                a.AddImpulse(-frictionImpulse);
                a.Rotate(-Vec2.Cross(ra, frictionImpulse));

                b.AddImpulse(frictionImpulse);
                b.Rotate(Vec2.Cross(rb, frictionImpulse));

                PositionCorrection(manifold);
            }
            
        }

        public static void ResolveCollisionWithRotation(Manifold manifold)
        {   
            Rigidbody a = manifold.a;
            Rigidbody b = manifold.b;
            Vec2 normal = manifold.normal;
            List<Vec2> contacts = manifold.contacts;

            for (int i = 0; i < manifold.contacts.Count; i++)
            {
                Scene.contacts.Add(manifold.contacts[i]);
            }

            float e = Math.Min(a.restitution, b.restitution);        
           
            for (int i = 0; i < contacts.Count; i++)
            {
                impulseList[i] = Vec2.zero;
                raList[i] = Vec2.zero;
                rbList[i] = Vec2.zero;
            }           

            for (int i = 0; i < contacts.Count; i++)
            {   
                Vec2 ra = contacts[i] - a.position;
                Vec2 rb = contacts[i] - b.position;

                raList[i] = ra;
                rbList[i] = rb;

                Vec2 raPerp = new Vec2(-ra.y, ra.x);
                Vec2 rbPerp = new Vec2(-rb.y, rb.x);

                Vec2 angularLinearVelocityA = raPerp * a.angularVelocity;
                Vec2 angularLinearVelocityB = rbPerp * b.angularVelocity;

                Vec2 relativeVelocity = (b.velocity + angularLinearVelocityB) - (a.velocity + angularLinearVelocityA);

                float contactVelocityMag = Vec2.Dot(relativeVelocity, normal);

                if (contactVelocityMag > 0.0f) continue;

                float raPerpDotN = Vec2.Dot(raPerp, normal);
                float rbPerpDotN = Vec2.Dot(rbPerp, normal);

                float denom = a.InversedMass + b.InversedMass + (raPerpDotN * raPerpDotN)
                    * a.InversedInertia + (rbPerpDotN * rbPerpDotN) * b.InversedInertia;

                float j = -(1f + e) * contactVelocityMag;
                j /= denom;
                j /= (float)contacts.Count;

                Vec2 impulse = j * normal;
                impulseList[i] = impulse;
            }

            for (int i = 0; i < contacts.Count; i++)
            {
                Vec2 impulse = impulseList[i];
                Vec2 ra = raList[i];
                Vec2 rb = rbList[i];

                a.AddImpulse(-impulse);
                a.Rotate(-Vec2.Cross(ra, impulse));

                b.AddImpulse(impulse);
                b.Rotate(Vec2.Cross(rb, impulse));
               
                PositionCorrection(manifold);
            }
        }

        private static void PositionCorrection(Manifold manifold)
        {
            Rigidbody a = manifold.a;
            Rigidbody b = manifold.b;

            const float percent = 0.5f;
            const float slop = 0.01f;
            Vec2 correction = Math.Max(manifold.penDepth - slop, 0f) / (a.InversedMass + b.InversedMass) * percent * manifold.normal;
            if (a.Mass != 0) a.position -= a.InversedMass * correction;
            if (b.Mass != 0) b.position += b.InversedMass * correction;
        }
    }
}
