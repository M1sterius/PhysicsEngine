using System;
using System.Diagnostics;

namespace PhysicsEngine2D
{
    public static class Time
    {
        public static float deltaTime;
        public static float deltaTimeMilliseconds;
        private static Stopwatch stopwatch = new();

        public static void UpdateTime()
        {
            stopwatch.Stop();
            deltaTimeMilliseconds = (float)stopwatch.Elapsed.TotalMilliseconds;
            deltaTime = deltaTimeMilliseconds / 1000;
            stopwatch.Reset();
            stopwatch.Start();
        }
    }
}
