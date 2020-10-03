package org.firstinspires.ftc.teamcode.math;

import static java.lang.Math.PI;

public class MathUtil {
    public static final double EPSILON = 1e-6;

    public static boolean approxEquals(double d1, double d2) {
        if (Double.isInfinite(d1)) {
            // Infinity - infinity is NaN, so we need a special case
            return d1 == d2;
        } else {
            return Math.abs(d1 - d2) < EPSILON;
        }
    }

    public static double angleWrap(double angle)
    {
        while (angle>PI) angle -= PI*2;
        while (angle<-PI) angle += PI*2;
        return angle;
    }

    public static double cosFromSin(double sin, double angle)
    {
        double cos = Math.sqrt(1-sin*sin);
        angle = angle % (PI*2);
        if ((angle>PI) && (angle<PI*2))
            cos *= -1;
        return cos;
    }
}
