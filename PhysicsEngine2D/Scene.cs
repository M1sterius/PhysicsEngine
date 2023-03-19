using Raylib_cs;
using System;
using System.Diagnostics;
using System.Numerics;

namespace PhysicsEngine2D
{
    public class Scene
    {   
        // bodies management
        private List<Rigidbody> bodies = new List<Rigidbody>();
        private List<Pair> pairs = new List<Pair>();

        public int BodiesCount
        {
            get { return bodies.Count; }
        }

        public int solvingIterations = 16;
        private bool isBroadPhasePassed;

        // physics
        Vec2 gravity = new Vec2(0f, 981f);

        // debug
        public bool showStats = true;
        public int statsFontSize = 16;
        public Color statsColor = Color.WHITE;
        public Vec2 statsPosition = new Vec2(10, 10);

        public bool drawContacts = false;
        public static List<Vec2> contacts = new List<Vec2>();

        public bool destroyObjectByVelocity = true;
        public float destroyingVelocity = 2000f;

        // timings
        public float physicsTime;
        public float renderTime;
        public float updateTime;

        private readonly Stopwatch physicsStopwatch = new Stopwatch();
        private readonly Stopwatch renderStopwatch = new Stopwatch();
        private readonly Stopwatch updateStopwatch = new Stopwatch();

        public void Update()
        {
            updateStopwatch.Start();

            DestroyBySpeed();

            PhysicsUpdate();
            Render();

            updateStopwatch.Stop();
            updateTime = (float)updateStopwatch.Elapsed.TotalMilliseconds;
            updateStopwatch.Reset();           
        }

        private void PhysicsUpdate()
        {
            physicsStopwatch.Start();

            float time = Time.deltaTime / solvingIterations;
            isBroadPhasePassed = false;           

            for (int i = 0; i < solvingIterations; i++)
            {
                contacts.Clear();
                for (int j = 0; j < bodies.Count; j++)
                {
                    Rigidbody rigidbody = bodies[j];
                    if (rigidbody.Mass == 0) continue;
                    rigidbody.rotation += rigidbody.angularVelocity * time;
                    rigidbody.Accelerate(gravity * rigidbody.gravityScale * time);
                    rigidbody.position += rigidbody.velocity * time;
                    if (rigidbody is Polygon polygon) polygon.UpdateVertices();
                }
                CollisionDetection();
            }

            physicsStopwatch.Stop();
            physicsTime = (float)physicsStopwatch.Elapsed.TotalMilliseconds;
            physicsStopwatch.Reset();           
        }

        private void CollisionDetection()
        {
            // Broad phase
            
            if (!isBroadPhasePassed)
            {
                pairs.Clear();

                for (int i = 0; i < bodies.Count - 1; i++)
                {   
                    for (int j = i + 1; j < bodies.Count; j++)
                    {
                        Rigidbody a = bodies[i];
                        Rigidbody b = bodies[j];
                        if (a.Mass == 0 && b.Mass == 0) continue;
                        if (Collision.AabbVsAabb(a.GetAABB(), b.GetAABB())) pairs.Add(new Pair(a, b));
                    }
                }
                isBroadPhasePassed = true;
            }

            // Narrow phase

            for (int n = 0; n < pairs.Count; n++)
            {
                Rigidbody a = pairs[n].rbA;
                Rigidbody b = pairs[n].rbB;

                if (a is Polygon polygonA && b is Polygon polygonB) Collision.PolygonVsPolygon(polygonA, polygonB);
                if (a is Circle circleA && b is Circle circleB) Collision.CircleVsCircle(circleA, circleB);
                if (a is Circle circle && b is Polygon polygon) Collision.CircleVsPolygon(circle, polygon);
                if (b is Circle circle1 && a is Polygon polygon1) Collision.CircleVsPolygon(circle1, polygon1);
            }
        }

        private void Render()
        {
            renderStopwatch.Start();

            for (int i = 0; i < bodies.Count; i++)
            {
                Rigidbody rb = bodies[i];
                if (rb is Circle circle)
                {
                    Raylib.DrawCircle((int)circle.position.x, (int)circle.position.y, circle.radius, circle.color);
                    if (circle.drawLine)
                    {
                        int x = (int)(circle.radius * Math.Sin(-circle.rotation  + Math.PI) + circle.position.x);
                        int y = (int)(circle.radius * Math.Cos(-circle.rotation + Math.PI) + circle.position.y);
                        Raylib.DrawLine((int)circle.position.x, (int)circle.position.y, x, y, Color.RED);
                    }
                }
                else if (rb is Polygon polygon)
                {
                    Rectangle rect = new Rectangle((int)polygon.position.x, (int)polygon.position.y, (int)polygon.size.x, (int)polygon.size.y);
                    float rotation = Mathf.RadiansToDegrees(polygon.rotation);
                    Raylib.DrawRectanglePro(rect, new Vector2(polygon.size.x / 2, polygon.size.y / 2), rotation, polygon.color);
                    if (polygon.drawVerices)
                    {
                        for (int j = 0; j < polygon.vertices.Length; j++)
                        {
                            Vec2 vert = polygon.vertices[j];
                            Raylib.DrawCircle((int)vert.x, (int)vert.y, 2f, Color.RED);
                        }
                    }
                }
            }

            if (drawContacts)
            {
                for (int i = 0; i < contacts.Count; i++)
                {
                    Vec2 contact = contacts[i];
                    Raylib.DrawCircle((int)contact.x, (int)contact.y, 3, Color.RED);
                }
            }

            if (showStats)
            {
                int x = (int)statsPosition.x;
                int y = (int)statsPosition.y;

                Raylib.DrawText($"Bodies: {BodiesCount}", x, y, statsFontSize, statsColor);
                Raylib.DrawText($"Physics: {physicsTime}", x, y + 16, statsFontSize, statsColor);
                Raylib.DrawText($"Render: {renderTime}", x, y + 32, statsFontSize, statsColor);
                Raylib.DrawText($"Update: {updateTime}", x, y + 48, statsFontSize, statsColor);
                Raylib.DrawText($"Frame: {Time.deltaTime}", x, y + 64, statsFontSize, statsColor);
            }

            renderStopwatch.Stop();
            renderTime = (float)renderStopwatch.Elapsed.TotalMilliseconds;
            renderStopwatch.Reset();           
        }

        private void DestroyBySpeed()
        {
            for (int b = 0; b < bodies.Count; b++)
            {   
                Rigidbody rigidbody = bodies[b];
                if (Vec2.Lenght(rigidbody.velocity) > destroyingVelocity) Destroy(rigidbody);
            }
        }

        public void Add(Rigidbody rb)
        {
            bodies.Add(rb);
            if (rb is Polygon polygon) polygon.UpdateVertices();
        }
        public void Destroy(Rigidbody rb)
        {
            try
            {
                bodies.Remove(rb);
            }
            catch (Exception)
            {

                Console.WriteLine("The object you are trying to destroy can not be found");
            }
        }

        public void SpawnCircleOnMouse()
        {
            if (Raylib.IsMouseButtonPressed(0))
            {
                Circle circle = new Circle(new Vec2(Raylib.GetMouseX(), Raylib.GetMouseY()), 30f);
                circle.color = Color.LIME;
                circle.Mass = 1f;
                circle.Inertia = 1000f;
                Add(circle);
            }
        }
        public void SpawnPolygonOnMouse()
        {
            if (Raylib.IsMouseButtonPressed((MouseButton)1))
            {
                Polygon polygon = new Polygon(new Vec2(Raylib.GetMouseX(), Raylib.GetMouseY()), new Vec2(50, 50));
                polygon.color = Color.VIOLET;
                //polygon.rotation = Mathf.RandomFloat(0, (float)Math.PI * 2);
                polygon.Inertia = 2000f;
                polygon.Mass = 1f;
                Add(polygon);
            }
        }
    }
}
