using FastTrianglelntersection;
using Kinematicula.Mathematics;

namespace FastTriangleIntersection.Tests;

public class FastTriangleIntersectionCheckTest
{
    
    [Fact]
    public void TriangleCollisionTest__Two_triangles_Simple_1__No_collision()
    {
        var pa1 = new Position3D(-10, 10, 0);
        var pa2 = new Position3D(10, 10, 0);
        var pa3 = new Position3D(0, -10, 0);
        var pb1 = new Position3D(-10, 00, 20);
        var pb2 = new Position3D(10, 00, 20);
        var pb3 = new Position3D(0, 00, 10);
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
        Assert.False(result);
    }

    [Fact]
    public void TriangleCollisionTest__Several_Two_triangles_Simple__No_collision()
    {
        var pa1 = new Position3D(-10, 10, 0);
        var pa2 = new Position3D(10, 10, 0);
        var pa3 = new Position3D(0, -10, 0);
        var pb1 = new Position3D(-10, 10, 0);
        var pb2 = new Position3D(10, 10, 0);
        var pb3 = new Position3D(0, -10, 0);
        var result1 = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
        pa1 = new Position3D(-10, 10, -10);
        pa2 = new Position3D(10, 10, -10);
        pa3 = new Position3D(0, -10, -10);
        pb1 = new Position3D(-10, 10, 10);
        pb2 = new Position3D(10, 10, 10);
        pb3 = new Position3D(0, -10, 10);
        var result2 = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
        pa1 = new Position3D(-10, 10, 0);
        pa2 = new Position3D(10, 10, 0);
        pa3 = new Position3D(0, -10, 0);
        pb1 = new Position3D(-10, 00, 20);
        pb2 = new Position3D(10, 00, 20);
        pb3 = new Position3D(0, 00, 0);
        var result4 = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);

        Assert.False(result1);
        Assert.False(result2);
        Assert.False(result4);
    }


    [Fact]
    public void TriangleCollisionTest__Two_triangles_Simple__No_collision()
    {
        var pa1 = new Position3D(-10, 10, 0);
        var pa2 = new Position3D(10, 10, 0);
        var pa3 = new Position3D(0, -10, 0);
        var pb1 = new Position3D(125.656034871287, -4.72812053605282, -5.26879345289812);
        var pb2 = new Position3D(179.666645529299, -7.62795129377592, -9.33006682921477);
        var pb3 = new Position3D(301.970915225248, -56.2496970392288, 14.3019008403036);
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);

        Assert.False(result);
    }


    [Fact]
    public void TriangleCollisionTest__Two_special_triangles__No_collision()
    {
        var pa1 = new Position3D(80, -50, 340);
        var pa2 = new Position3D(80, 50, 340);
        var pa3 = new Position3D(76.0845213036123, -50, 364.721359549996);
        var pb1 = new Position3D(80, 50, 340);
        var pb2 = new Position3D(76.0845213036123, 50, 315.278640450004);
        var pb3 = new Position3D(80, 150, 340);
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);

        Assert.False(result);
    }


    [Fact]
    public void TriangleCollisionTest__Two_triangles_Simple__No_collision2()
    {
        var pa1 = new Position3D(-10, 10, 0);
        var pa2 = new Position3D(10, 10, 0);
        var pa3 = new Position3D(0, -10, 0);
        var pb1 = new Position3D(125, -4, -5);
        var pb2 = new Position3D(179, -7, -9);
        var pb3 = new Position3D(301, -56, 14);
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);

        Assert.False(result);
    }


    [Fact]
    public void TriangleCollisionTest__Several_Two_triangles_Simple__Collision()
    {
        var pa1 = new Position3D(-10, 10, 0);
        var pa2 = new Position3D(10, 10, 0);
        var pa3 = new Position3D(0, -10, 0);
        var pb1 = new Position3D(-10, 00, 10);
        var pb2 = new Position3D(10, 00, 10);
        var pb3 = new Position3D(0, 00, -10);
        var result1 = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
        pa1 = new Position3D(-10, 10, 0);
        pa2 = new Position3D(10, 10, 0);
        pa3 = new Position3D(0, -10, 0);
        pb1 = new Position3D(-12, 00, 10);
        pb2 = new Position3D(8, 00, 10);
        pb3 = new Position3D(-2, 00, -10);
        var result2 = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);

        Assert.True(result1);
        Assert.True(result2);
    }


    [Fact]
    public void TriangleCollisionTest__Random_two_triangles__Collision()
    {
        var pa1 = new Position3D(-10, 10, 0);
        var pa2 = new Position3D(10, 10, 0);
        var pa3 = new Position3D(0, -10, 0);
        var n = 100000;
        for (int i = 0; i < n; i++)
        {
            var rand = new Random();
            var translation = new Position3D(rand.NextDouble(), rand.NextDouble(), 0.0);
            var normal = new Vector3D(rand.NextDouble(), rand.NextDouble(), rand.NextDouble());
            if (normal.Length == 0.0)
            {
                normal = new Vector3D(0, 1, 0);
            }
            if (normal == new Vector3D(0, 0, 1))
            {
                normal = new Vector3D(0, 1, 0);
            }
            normal.Normalize();
            var matrix = Matrix44D.CreatePlaneCoordinateSystem(translation, normal);
            var pb1 = matrix * pa1;
            var pb2 = matrix * pa2;
            var pb3 = matrix * pa3;
            var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
            Assert.True(result);
        }
    }


    [Fact]
    public void TriangleCollisionTest__Two_special_triangles__Collision()
    {
        var pa1 = new Position3D(-10, 10, 0);
        var pa2 = new Position3D(10, 10, 0);
        var pa3 = new Position3D(0, -10, 0);
        var pb1 = new Position3D(-10, 10, -5);
        var pb2 = new Position3D(10, 10, -5);
        var pb3 = new Position3D(0, -7, 5);
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);

        Assert.True(result);
    }


    [Fact]
    public void TriangleCollisionTest__Two_special_triangles_2__Collision()
    {
        var pa1 = new Position3D(-10, 10, 0);
        var pa2 = new Position3D(10, 10, 0);
        var pa3 = new Position3D(0, -10, 0);
        var pb1 = new Position3D(-6, 8, -9);
        var pb2 = new Position3D(6, -6, -10);
        var pb3 = new Position3D(0.0, -0, 10);
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);

        Assert.True(result);
    }

    [Fact]
    public void TriangleCollisionTest__Two_special_triangles_3__Collision()
    {
        var pa1 = new Position3D(-10, 10, 0);
        var pa2 = new Position3D(10, 10, 0);
        var pa3 = new Position3D(0, -10, 0);
        var pb1 = new Position3D(-87, 72, -10);
        var pb2 = new Position3D(-125, 103, -29);
        var pb3 = new Position3D(-121, 87, 20);
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
        Assert.False(result);
    }

    [Fact]
    public void TriangleCollisionTest__Two_special_triangles_4__Collision()
    {
        var pa1 = new Position3D(80, 50, 340);
        var pa2 = new Position3D(80, -50, 340);
        var pa3 = new Position3D(80, -50, 40);
        var pb1 = new Position3D(85, -90, 312);
        var pb3 = new Position3D(85, -50, 312);
        var pb2 = new Position3D(72, -50, 287);
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
        Assert.False(result);
    }


    [Fact]
    public void TriangleCollisionTest__Two_special_triangles_5__Collision()
    {
        var pa1 = new Position3D(80, 50, 340);
        var pa2 = new Position3D(80, -50, 340);
        var pa3 = new Position3D(80, -50, 40);
        var pb1 = new Position3D(85.595086466563814, -90, 312.1884705062547);
        var pb2 = new Position3D(72.811529493745255, -50, 287.09932729367739);
        var pb3 = new Position3D(85.595086466563814, -50, 312.1884705062547);
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
        Assert.False(result);
    }


    [Fact]
    public void TriangleCollisionTest_Two_special_triangles_6__Collision()
    {
        var pa1 = new Position3D(436.257463338708, -68.6345591573637, 476.038015232257);
        var pa2 = new Position3D(435.622097039279, -68.5302417834636, 456.048382230347);
        var pa3 = new Position3D(432.028218101821, -53.5751591635402, 476.25102949595);
        var pb1 = new Position3D(450.67539350746, -107.691603872714, 455.365548498067);
        var pb2 = new Position3D(458.51387084774, -68.4671689189287, 455.321101426556);
        var pb3 = new Position3D(321.300321858654, -41.041776957956, 459.825526984479);
        var ex = pa2 - pa1;
        var ey = pa3 - pa1;
        var ez = (ex & ey).Normalize();
        ey = (ez & ex).Normalize();
        ex.Normalize();
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
        Assert.False(result);
    }


    [Fact]
    public void TriangleCollisionTest__Two_special_triangles_7__Collision()
    {
        var pa1 = new Position3D(386.998441813186, 15.4508497187474, 447.05852357877);
        var pa2 = new Position3D(406.404535283603, 15.4508497187474, 442.220803432105);
        var pa3 = new Position3D(386.406504620269, 0.0, 444.684019029909);
        var pb1 = new Position3D(434.839342865156, -20, 556.282761782994);
        var pb2 = new Position3D(434.839342865156, 20, 556.282761782994);
        var pb3 = new Position3D(400.974591555991, 20, 420.440284557465);
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
        Assert.False(result);
    }


    [Fact]
    public void TriangleCollisionTest__Two_special_triangles_8__Collision()
    {
        var pa1 = new Position3D(447.449792053125, 67.2325730332692, 482.045073192544);
        var pa2 = new Position3D(468.575324677108, 55.8955338579372, 473.684401412462);
        var pa3 = new Position3D(462.37305590072, 69.4415921096815, 468.914329464364);
        var pb1 = new Position3D(487.228608370181, 21.6247296984552, 489.118817037138);
        var pb2 = new Position3D(482.810412490474, 61.3452628598734, 490.779777765515);
        var pb3 = new Position3D(390.895296930189, 55.5304954963434, 385.338798957238);
        var ex = pb2 - pb1;
        var ey = pb3 - pb1;
        var ez = (ex & ey).Normalize();
        ey = (ez & ex).Normalize();
        ex.Normalize();
        var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
        Assert.True(result);
    }


    [Fact]
    public void TriangleCollisionTest__Random_two_triangles__No_collision()
    {
        var pa1 = new Position3D(-10, 10, 0);
        var pa2 = new Position3D(10, 10, 0);
        var pa3 = new Position3D(0, -10, 0);
        var n = 10000;
        for (int i = 0; i < n; i++)
        {
            var rand = new Random();
            var angle = rand.NextDouble() * 2.0 * Math.PI;
            var radius = 20.0 + rand.NextDouble() * 100.0;
            var rotation = Matrix44D.CreateRotation(new Position3D(0, 0, 0), new Vector3D(0, 0, 1), angle);
            var translation = rotation * new Position3D(radius, 0, 0);
            var normal = new Vector3D(rand.NextDouble(), rand.NextDouble(), rand.NextDouble());
            if (normal.Length == 0.0)
            {
                normal = new Vector3D(0, 1, 0);
            }
            if (normal == new Vector3D(0, 0, 1))
            {
                normal = new Vector3D(0, 1, 0);
            }
            normal.Normalize();
            var matrix = Matrix44D.CreatePlaneCoordinateSystem(translation, normal);
            var pb1 = matrix * pa1 * (1.0 + rand.NextDouble() * 2.0);
            var pb2 = matrix * pa2 * (1.0 + rand.NextDouble() * 2.0);
            var pb3 = matrix * pa3 * (1.0 + rand.NextDouble() * 2.0);
            var result = TriangleCollisionTest(pa1, pa2, pa3, pb1, pb2, pb3);
            Assert.False(result);
        }
    }

    private bool TriangleCollisionTest(
        Position3D pa1, Position3D pa2, Position3D pa3,
        Position3D pb1, Position3D pb2, Position3D pb3)
    {
        bool collision
            = FastTriangleIntersectionCheck.TriangleCollisionTest(pa1.X, pa1.Y, pa1.Z,
                                                                  pa2.X, pa2.Y, pa2.Z,
                                                                  pa3.X, pa3.Y, pa3.Z,
                                                                  pb1.X, pb1.Y, pb1.Z,
                                                                  pb2.X, pb2.Y, pb2.Z,
                                                                  pb3.X, pb3.Y, pb3.Z);
        return collision;
    }
}