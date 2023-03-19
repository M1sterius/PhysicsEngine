using Raylib_cs;
using System;

namespace PhysicsEngine2D
{
    public abstract class Rigidbody
    {
        // transform
        public Vec2 position;
        public Vec2 velocity;
        public float rotation;
        public float angularVelocity;

        // physical parametres
        public float gravityScale = 1f;
        public float restitution = 0.5f;
        public float staticFriction = 0.3f;
        public float dynamicFriction = 0.1f;

        private float mass = 1f;
        private float inertia = 1f;

        private float inversedMass = 1f;
        private float inversedInertia = 1f;

        // appearance
        public Color color = Color.BEIGE;

        public Rigidbody(Vec2 position)
        {
            this.position = position;
        }

        public float Mass
        {
            get { return mass; }
            set
            {
                mass = value;
                if (mass == 0) inversedMass = 0;
                else inversedMass = 1 / mass;
            }
        }
        public float Inertia
        {
            get { return inertia; }
            set
            {
                inertia = value;
                if (inertia == 0) inversedInertia = 0f;
                else inversedInertia = 1f / inertia;
            }
        }
        public float InversedMass
        {
            get { return inversedMass; }
            set
            {
                inversedMass = value;
                if (inversedMass == 0) mass = 0;
                else mass = 1 / inversedMass;
            }
        }
        public float InversedInertia
        {
            get { return inversedInertia; }
            set
            {
                inversedInertia = value;
                if (inversedInertia == 0) inertia = 0;
                else inertia = 1 / inversedInertia;
            }
        }

        public virtual AABB GetAABB()
        {
            return new AABB(position, Vec2.zero);
        }

        public void Translate(Vec2 translation)
        {
            position += translation;
        }
        public void Accelerate(Vec2 acceleration)
        {
            if (mass == 0) return;
            velocity += acceleration;
        }
        public void AddForce(Vec2 force)
        {
            if (mass == 0) return;
            velocity += force * inversedMass;
        }
        public void AddImpulse(Vec2 impulse)
        {
            if (mass == 0) return;
            velocity += impulse * inversedMass;
        }
        public void Rotate(float rotation)
        {
            if (inertia == 0) return;
            this.angularVelocity += rotation * inversedInertia;
        }
    }

    public class Polygon : Rigidbody
    {
        public readonly Vec2 size;
        public bool drawVerices = false;

        private Vec2[] localVerices = new Vec2[4];
        public Vec2[] vertices = new Vec2[4];

        public Polygon(Vec2 position, Vec2 size) : base(position)
        {
            this.size = size;

            localVerices[0] = new Vec2(-size.x / 2, -size.y / 2);
            localVerices[1] = new Vec2(size.x / 2, -size.y / 2);
            localVerices[2] = new Vec2(size.x / 2, size.y / 2);
            localVerices[3] = new Vec2(-size.x / 2, size.y / 2);
            UpdateVertices();
        }
        public override AABB GetAABB()
        {
            float minX = float.MaxValue;
            float minY = float.MaxValue;
            float maxX = float.MinValue;
            float maxY = float.MinValue;

            for (int i = 0; i < vertices.Length; i++)
            {
                Vec2 v = vertices[i];

                if (v.x < minX) minX = v.x;
                if (v.x > maxX) maxX = v.x;

                if (v.y < minY) minY = v.y;
                if (v.y > maxY) maxY = v.y;
            }

            return new AABB(new Vec2(minX, minY), new Vec2(maxX, maxY));
        }
        public void UpdateVertices()
        {
            for (int i = 0; i < vertices.Length; i++)
            {
                Vec2 vert = vertices[i];
                Vec2 localVert = localVerices[i];
                double x = (localVert.x * Math.Cos(rotation) - localVert.y * Math.Sin(rotation)) + position.x;
                double y = (localVert.x * Math.Sin(rotation) + localVert.y * Math.Cos(rotation)) + position.y;
                vertices[i] = new Vec2((float)x, (float)y);
            }
        }
    }

    public class Circle : Rigidbody
    {
        public float radius;
        public bool drawLine = true;

        public Circle(Vec2 position, float radius) : base(position)
        {
            this.radius = radius;
        }
        public override AABB GetAABB()
        {
            return new AABB(new Vec2(position.x - radius, position.y - radius), new Vec2(position.x + radius, position.y + radius));
        }
    }
}
