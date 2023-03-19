using Raylib_cs;

namespace PhysicsEngine2D
{
    public static class Program
    {
        public static void Main()
        {   
            Scene scene = new Scene();

            Polygon polygon = new Polygon(new Vec2(640, 680), new Vec2(1280, 40));
            polygon.Mass = 0f;
            polygon.Inertia = 0f;
            scene.Add(polygon);

            Polygon polygon1 = new Polygon(new Vec2(340, 280), new Vec2(600, 40));
            polygon1.Mass = 0f;
            polygon1.Inertia = 0f;
            polygon1.rotation = 0.3f;
            scene.Add(polygon1);

            Polygon polygon2 = new Polygon(new Vec2(40, 360), new Vec2(40, 720));
            polygon2.Mass = 0f;
            polygon2.Inertia = 0f;
            scene.Add(polygon2);

            Polygon polygon3 = new Polygon(new Vec2(1240, 360), new Vec2(40, 720));
            polygon3.Mass = 0f;
            polygon3.Inertia = 0f;
            scene.Add(polygon3);

            Circle circle = new Circle(new Vec2(900, 380), 100);
            circle.Mass = 0f;
            circle.Inertia = 0f;
            scene.Add(circle);

            Raylib.InitWindow(1280, 720, "Physics Engine");
            Raylib.SetTargetFPS(165);

            while (!Raylib.WindowShouldClose())
            {
                Time.UpdateTime();                

                Raylib.BeginDrawing();
                Raylib.ClearBackground(Color.BLACK);

                scene.SpawnCircleOnMouse();
                scene.SpawnPolygonOnMouse();

                scene.Update();

                Raylib.EndDrawing();
            }

            Raylib.CloseWindow();
        }
    }
}
