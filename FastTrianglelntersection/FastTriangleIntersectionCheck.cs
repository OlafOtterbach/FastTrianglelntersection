namespace FastTrianglelntersection;
public static class FastTriangleIntersectionCheck
{
    /// <summary>
    /// Checks if two triangles have a collision together.
    /// </summary>
    /// <param name="a1x">X coordinate of point 1 of first  triangle A</param>
    /// <param name="a1y">Y coordinate of point 1 of first  triangle A</param>
    /// <param name="a1z">Z coordinate of point 1 of first  triangle A</param>
    /// <param name="a2x">X coordinate of point 2 of first  triangle A</param>
    /// <param name="a2y">Y coordinate of point 2 of first  triangle A</param>
    /// <param name="a2z">Z coordinate of point 2 of first  triangle A</param>
    /// <param name="a3x">X coordinate of point 3 of first  triangle A</param>
    /// <param name="a3y">Y coordinate of point 3 of first  triangle A</param>
    /// <param name="a3z">Z coordinate of point 3 of first  triangle A</param>
    /// <param name="b1x">X coordinate of point 1 of second triangle B</param>
    /// <param name="b1y">Y coordinate of point 1 of second triangle B</param>
    /// <param name="b1z">Z coordinate of point 1 of second triangle B</param>
    /// <param name="b2x">X coordinate of point 2 of second triangle B</param>
    /// <param name="b2y">Y coordinate of point 2 of second triangle B</param>
    /// <param name="b2z">Z coordinate of point 2 of second triangle B</param>
    /// <param name="b3x">X coordinate of point 3 of second triangle B</param>
    /// <param name="b3y">Y coordinate of point 3 of second triangle B</param>
    /// <param name="b3z">Z coordinate of point 3 of second triangle B</param>
    /// <returns>true=collision, false=no collision or touch in same plane</returns>
    public static bool TriangleCollisionTest
    (
       double a1x, double a1y, double a1z,
       double a2x, double a2y, double a2z,
       double a3x, double a3y, double a3z,
       double b1x, double b1y, double b1z,
       double b2x, double b2y, double b2z,
       double b3x, double b3y, double b3z
    )
    {
        const double epsilon = 0.0001;
        bool r_result = true;

        // Determine if A intersects B
        //

        // calculate HNF B
        double b12x = b2x - b1x; double b12y = b2y - b1y; double b12z = b2z - b1z;
        double b13x = b3x - b1x; double b13y = b3y - b1y; double b13z = b3z - b1z;
        double nbx = b12y * b13z - b13y * b12z;
        double nby = b12z * b13x - b13z * b12x;
        double nbz = b12x * b13y - b13x * b12y;
        double db = -nbx * b1x - nby * b1y - nbz * b1z;

        // calculate distances
        double da1 = nbx * a1x + nby * a1y + nbz * a1z + db;
        double da2 = nbx * a2x + nby * a2y + nbz * a2z + db;
        double da3 = nbx * a3x + nby * a3y + nbz * a3z + db;

        double da1Val = (Math.Abs(da1) < epsilon) ? 0.0 : da1;
        double da2Val = (Math.Abs(da2) < epsilon) ? 0.0 : da2;
        double da3Val = (Math.Abs(da3) < epsilon) ? 0.0 : da3;
        r_result = !(((da1Val >= 0.0) && (da2Val >= 0.0) && (da3Val >= 0.0))
                      || ((da1Val <= 0.0) && (da2Val <= 0.0) && (da3Val <= 0.0))
                    );

        // Determine if B intersects A
        //
        if (r_result)
        {
            // calculate HNF A
            double a12x = a2x - a1x; double a12y = a2y - a1y; double a12z = a2z - a1z;
            double a13x = a3x - a1x; double a13y = a3y - a1y; double a13z = a3z - a1z;
            double nax = a12y * a13z - a13y * a12z;
            double nay = a12z * a13x - a13z * a12x;
            double naz = a12x * a13y - a13x * a12y;
            double da = -nax * a1x - nay * a1y - naz * a1z;

            // calculate distances
            double db1 = nax * b1x + nay * b1y + naz * b1z + da;
            double db2 = nax * b2x + nay * b2y + naz * b2z + da;
            double db3 = nax * b3x + nay * b3y + naz * b3z + da;
            double db1Val = (Math.Abs(db1) < epsilon) ? 0.0 : db1;
            double db2Val = (Math.Abs(db2) < epsilon) ? 0.0 : db2;
            double db3Val = (Math.Abs(db3) < epsilon) ? 0.0 : db3;
            r_result = !(((db1Val >= 0.0) && (db2Val >= 0.0) && (db3Val >= 0.0))
                          || ((db1Val <= 0.0) && (db2Val <= 0.0) && (db3Val <= 0.0))
                        );

            // Calculates intersection
            if (r_result)
            {
                // Richtung Schnittlinie zwischen Ebenen
                double Dx = Math.Abs(nay * nbz - nby * naz);
                double Dy = Math.Abs(naz * nbx - nbz * nax);
                double Dz = Math.Abs(nax * nby - nbx * nay);

                // Koordinatenachse bestimmen
                double t1, t2, t3, t4;
                if (Dx >= Dy)
                {
                    if (Dx >= Dz)
                    {
                        if ((da2 > 0.0 && da3 > 0.0) || (da2 < 0.0 && da3 < 0.0))
                        {
                            t1 = a2x + (a1x - a2x) * da2 / (da2 - da1);
                            t2 = a3x + (a1x - a3x) * da3 / (da3 - da1);
                        }
                        else
                        {
                            if ((da2 > 0.0 && da1 > 0.0) || (da2 < 0.0 && da1 < 0.0))
                            {
                                t1 = a1x + (a3x - a1x) * da1 / (da1 - da3);
                                t2 = a2x + (a3x - a2x) * da2 / (da2 - da3);
                            }
                            else
                            {
                                t1 = a1x + (a2x - a1x) * da1 / (da1 - da2);
                                t2 = a3x + (a2x - a3x) * da3 / (da3 - da2);
                            }
                        }
                        if ((db2 > 0.0 && db3 > 0.0) || (db2 < 0.0 && db3 < 0.0))
                        {
                            t3 = b2x + (b1x - b2x) * db2 / (db2 - db1);
                            t4 = b3x + (b1x - b3x) * db3 / (db3 - db1);
                        }
                        else
                        {
                            if ((db2 > 0.0 && db1 > 0.0) || (db2 < 0.0 && db1 < 0.0))
                            {
                                t3 = b1x + (b3x - b1x) * db1 / (db1 - db3);
                                t4 = b2x + (b3x - b2x) * db2 / (db2 - db3);
                            }
                            else
                            {
                                t3 = b1x + (b2x - b1x) * db1 / (db1 - db2);
                                t4 = b3x + (b2x - b3x) * db3 / (db3 - db2);
                            }
                        }
                    }
                    else
                    {
                        if ((da2 > 0.0 && da3 > 0.0) || (da2 < 0.0 && da3 < 0.0))
                        {
                            t1 = a2z + (a1z - a2z) * da2 / (da2 - da1);
                            t2 = a3z + (a1z - a3z) * da3 / (da3 - da1);
                        }
                        else
                        {
                            if ((da2 > 0.0 && da1 > 0.0) || (da2 < 0.0 && da1 < 0.0))
                            {
                                t1 = a1z + (a3z - a1z) * da1 / (da1 - da3);
                                t2 = a2z + (a3z - a2z) * da2 / (da2 - da3);
                            }
                            else
                            {
                                t1 = a1z + (a2z - a1z) * da1 / (da1 - da2);
                                t2 = a3z + (a2z - a3z) * da3 / (da3 - da2);
                            }
                        }
                        if ((db2 > 0.0 && db3 > 0.0) || (db2 < 0.0 && db3 < 0.0))
                        {
                            t3 = b2z + (b1z - b2z) * db2 / (db2 - db1);
                            t4 = b3z + (b1z - b3z) * db3 / (db3 - db1);
                        }
                        else
                        {
                            if ((db2 > 0.0 && db1 > 0.0) || (db2 < 0.0 && db1 < 0.0))
                            {
                                t3 = b1z + (b3z - b1z) * db1 / (db1 - db3);
                                t4 = b2z + (b3z - b2z) * db2 / (db2 - db3);
                            }
                            else
                            {
                                t3 = b1z + (b2z - b1z) * db1 / (db1 - db2);
                                t4 = b3z + (b2z - b3z) * db3 / (db3 - db2);
                            }
                        }
                    }
                }
                else
                {
                    if (Dy >= Dz)
                    {
                        if ((da2 > 0.0 && da3 > 0.0) || (da2 < 0.0 && da3 < 0.0))
                        {
                            t1 = a2y + (a1y - a2y) * da2 / (da2 - da1);
                            t2 = a3y + (a1y - a3y) * da3 / (da3 - da1);
                        }
                        else
                        {
                            if ((da2 > 0.0 && da1 > 0.0) || (da2 < 0.0 && da1 < 0.0))
                            {
                                t1 = a1y + (a3y - a1y) * da1 / (da1 - da3);
                                t2 = a2y + (a3y - a2y) * da2 / (da2 - da3);
                            }
                            else
                            {
                                t1 = a1y + (a2y - a1y) * da1 / (da1 - da2);
                                t2 = a3y + (a2y - a3y) * da3 / (da3 - da2);
                            }
                        }
                        if ((db2 > 0.0 && db3 > 0.0) || (db2 < 0.0 && db3 < 0.0))
                        {
                            t3 = b2y + (b1y - b2y) * db2 / (db2 - db1);
                            t4 = b3y + (b1y - b3y) * db3 / (db3 - db1);
                        }
                        else
                        {
                            if ((db2 > 0.0 && db1 > 0.0) || (db2 < 0.0 && db1 < 0.0))
                            {
                                t3 = b1y + (b3y - b1y) * db1 / (db1 - db3);
                                t4 = b2y + (b3y - b2y) * db2 / (db2 - db3);
                            }
                            else
                            {
                                t3 = b1y + (b2y - b1y) * db1 / (db1 - db2);
                                t4 = b3y + (b2y - b3y) * db3 / (db3 - db2);
                            }
                        }

                    }
                    else
                    {
                        if ((da2 > 0.0 && da3 > 0.0) || (da2 < 0.0 && da3 < 0.0))
                        {
                            t1 = a2z + (a1z - a2z) * da2 / (da2 - da1);
                            t2 = a3z + (a1z - a3z) * da3 / (da3 - da1);
                        }
                        else
                        {
                            if ((da2 > 0.0 && da1 > 0.0) || (da2 < 0.0 && da1 < 0.0))
                            {
                                t1 = a1z + (a3z - a1z) * da1 / (da1 - da3);
                                t2 = a2z + (a3z - a2z) * da2 / (da2 - da3);
                            }
                            else
                            {
                                t1 = a1z + (a2z - a1z) * da1 / (da1 - da2);
                                t2 = a3z + (a2z - a3z) * da3 / (da3 - da2);
                            }
                        }
                        if ((db2 > 0.0 && db3 > 0.0) || (db2 < 0.0 && db3 < 0.0))
                        {
                            t3 = b2z + (b1z - b2z) * db2 / (db2 - db1);
                            t4 = b3z + (b1z - b3z) * db3 / (db3 - db1);
                        }
                        else
                        {
                            if ((db2 > 0.0 && db1 > 0.0) || (db2 < 0.0 && db1 < 0.0))
                            {
                                t3 = b1z + (b3z - b1z) * db1 / (db1 - db3);
                                t4 = b2z + (b3z - b2z) * db2 / (db2 - db3);
                            }
                            else
                            {
                                t3 = b1z + (b2z - b1z) * db1 / (db1 - db2);
                                t4 = b3z + (b2z - b3z) * db3 / (db3 - db2);
                            }
                        }
                    }
                }
                if (t1 > t2)
                {
                    var help = t1;
                    t1 = t2;
                    t2 = help;
                }
                if (t3 > t4)
                {
                    var help = t3;
                    t3 = t4;
                    t4 = help;
                }
                var dist12 = Math.Abs(t1 - t2);
                if (dist12 < 0.001) return false;
                var dist34 = Math.Abs(t3 - t4);
                if (dist34 < 0.001) return false;
                double t1min = t1 + epsilon;
                double t2max = t2 - epsilon;
                double t3min = t3 + epsilon;
                double t4max = t4 - epsilon;
                r_result = !((t2max < t3min) || (t1min > t4max));
            }
        }
        if (r_result)
        {
            var a = 0;
            a++;
        }
        return r_result;
    }
}
