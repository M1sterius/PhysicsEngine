using System;

namespace PhysicsEngine2D
{
    public static class Mathf
    {   
        public static readonly float floatComparisonSmallValue = 0.0005f;

        public static float DegreesToRadians(float degrees)
        {
            return (float)(Math.PI / 180) * degrees;
        }
        public static float RadiansToDegrees(float radians)
        {
            return (float)(180 / Math.PI) * radians;
        }
        public static float RandomFloat(float min, float max)
        {
            Random random = new Random();
            double val = (random.NextDouble() * (max - min) + min);
            return (float)val;
        }
        public static void ProjectVertices(List<Vec2> vertices, Vec2 axis, out float min, out float max)
        {
            min = float.MaxValue;
            max = float.MinValue;

            for (int i = 0; i < vertices.Count; i++)
            {
                Vec2 v = vertices[i];
                float projection = Vec2.Dot(v, axis);

                if (projection < min) { min = projection; }
                if (projection > max) { max = projection; }
            }
        }
        public static void ProjectCircle(Circle circle, Vec2 axis, out float min, out float max)
        {
            Vec2 direction = Vec2.Normalize(axis);
            Vec2 directionAndRadius = direction * circle.radius;

            Vec2 p1 = circle.position + directionAndRadius;
            Vec2 p2 = circle.position - directionAndRadius;

            min = Vec2.Dot(p1, axis);
            max = Vec2.Dot(p2, axis);

            if (min > max)
            {
                float t = min;
                min = max;
                max = t;
            }
        }
        public static bool NearlyEqual(float a, float b)
        {
            return Math.Abs(a - b) < Mathf.floatComparisonSmallValue;
        }
    }
}
