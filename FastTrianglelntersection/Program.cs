using static FastTrianglelntersection.FastTriangleIntersectionCheck;

Console.WriteLine("checking two triangles.");
Console.WriteLine();
Console.WriteLine("triangle 1");
Console.WriteLine("P1(-10, 10, 0)");
Console.WriteLine("P2( 10, 10, 0)");
Console.WriteLine("P3(  0,-10, 0)");
Console.WriteLine();
Console.WriteLine("triangle 1");
Console.WriteLine("P1(-10,  0, 10)");
Console.WriteLine("P2( 10,  0, 10)");
Console.WriteLine("P3(  0,  0,-10)");
Console.WriteLine();

var result
    = TriangleCollisionTest(
        // first triangle
        -10, 10, 0,
        10, 10, 0,
        0, -10, 0,

        // second triangle
        -10, 0, 10,
        10, 0, 10,
        0, 0, -10);

Console.WriteLine($"collission detected: {result}");

Console.ReadKey();
