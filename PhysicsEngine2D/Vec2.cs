using System;
using System.Numerics;

namespace PhysicsEngine2D
{
    public readonly struct Vec2
    {
        public readonly float x;
        public readonly float y;

        public static readonly Vec2 zero = new Vec2(0f, 0f);
        public static readonly Vec2 up = new Vec2(0f, -1f);
        public static readonly Vec2 down = new Vec2(0f, 1f);
        public static readonly Vec2 left = new Vec2(-1f, 0f);
        public static readonly Vec2 right = new Vec2(1f, 0f);

        public Vec2(float x, float y) { this.x = x; this.y = y; }

        public static Vec2 operator +(Vec2 a, Vec2 b)
        {
            return new Vec2(a.x + b.x, a.y + b.y);
        }
        public static Vec2 operator -(Vec2 a, Vec2 b)
        {
            return new Vec2(a.x - b.x, a.y - b.y);
        }
        public static Vec2 operator -(Vec2 v)
        {
            return new Vec2(-v.x, -v.y);
        }
        public static Vec2 operator *(Vec2 a, float s)
        {
            return new Vec2(a.x * s, a.y * s);
        }
        public static Vec2 operator *(float s, Vec2 a)
        {
            return new Vec2(a.x * s, a.y * s);
        }
        public static Vec2 operator /(Vec2 a, float s)
        {
            return new Vec2(a.x / s, a.y / s);
        }
        public static bool operator ==(Vec2 a, Vec2 b)
        {
            if (Mathf.NearlyEqual(a.x, b.x) && Mathf.NearlyEqual(a.y, b.y)) return true;
            return false;
        }
        public static bool operator !=(Vec2 a, Vec2 b)
        {
            if (!Mathf.NearlyEqual(a.x, b.x) || !Mathf.NearlyEqual(a.y, b.y)) return true;
            return false;
        }       

        public static bool NearlyEqual(Vec2 a, Vec2 b)
        {
            return Mathf.NearlyEqual(a.x, b.x) && Mathf.NearlyEqual(a.y, b.y);
        }
        public override string ToString()
        {
            return $"Vec2({this.x}, {this.y})";
        }
        public Vector2 ToSysNumericsVector2()
        {
            return new Vector2(this.x, this.y);
        }
        public static float Lenght(Vec2 v)
        {
            return (float)Math.Sqrt(v.x * v.x + v.y * v.y);
        }
        public static float LenghtSquared(Vec2 v)
        {
            return v.x * v.x + v.y * v.y;
        }
        public static float Distance(Vec2 a, Vec2 b)
        {
            float dX = a.x - b.x;
            float dY = a.y - b.y;
            return (float)Math.Sqrt(dX * dX + dY * dY);
        }
        public static float DistanceSquared(Vec2 a, Vec2 b)
        {
            float dX = a.x - b.x;
            float dY = a.y - b.y;
            return dX * dX + dY * dY;
        }
        public static Vec2 Normalize(Vec2 v)
        {
            float mag = Lenght(v);
            return new Vec2(v.x / mag, v.y / mag);
        }
        public Vec2 Normalized()
        {
            float mag = Lenght(this);
            return new Vec2(x / mag, y / mag);
        }
        public static float Dot(Vec2 a, Vec2 b)
        {
            return a.x * b.x + a.y * b.y;
        }
        public static float Cross(Vec2 a, Vec2 b)
        {
            return a.x * b.y - a.y * b.x;
        }
        public static Vec2 Rotate(Vec2 v, float angle)
        {
            return Vec2.zero;
        }
        public static Vec2 FindClosestPoint(Vec2 point, List<Vec2> points)
        {
            if (points.Count == 0) return point;
            float bigDist = float.MaxValue;
            int index = 0;
            for (int i = 0; i < points.Count; i++)
            {
                float tempDist = Vec2.Distance(point, points[i]);
                if (tempDist < bigDist) { bigDist = tempDist; index = i; }
            }
            return points[index];
        }
        public static void PointToSegmentDistance(Vec2 p, Vec2 a, Vec2 b, out float distanceSquared, out Vec2 cp)
        {
            Vec2 ab = b - a;
            Vec2 ap = p - a;

            float proj = Vec2.Dot(ap, ab);
            float abLenSq = Vec2.LenghtSquared(ab);
            float d = proj / abLenSq;

            if (d <= 0) cp = a;
            else if (d >= 1f) cp = b;
            else cp = a + ab * d;

            distanceSquared = Vec2.DistanceSquared(p, cp);
        }
    }
}
