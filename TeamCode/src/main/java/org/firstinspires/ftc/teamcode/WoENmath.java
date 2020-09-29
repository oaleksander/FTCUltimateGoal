package org.firstinspires.ftc.teamcode;
import static java.lang.Math.PI;

public class WoENmath {

    public static float AngleTransform(float angle)
    {
        while (angle>180) angle -= 360;
        while (angle<-180) angle += 360;
        return angle;
    }
    public static float AngleTransform360(float angle)
    {
        while (angle>360) angle -= 360;
        while (angle<0) angle += 360;
        return angle;
    }

    public static double AngleTransformPI(double angle)
    {
        while (angle>PI) angle -= PI*2;
        while (angle<-PI) angle += PI*2;
        return angle;
    }
    public static double AngleTransformTAU(double angle)
    {
        while (angle>PI*2) angle -= PI*2;
        while (angle<0) angle += PI*2;
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
