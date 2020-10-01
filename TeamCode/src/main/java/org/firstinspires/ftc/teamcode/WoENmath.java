package org.firstinspires.ftc.teamcode;
import static java.lang.Math.PI;

public class WoENmath {

    public class Point implements Cloneable {

        public double x, y, rot;

        public Point(double x, double y, double rot) {
            this.x = x;
            this.y = y;
            this.rot = angleTransform(rot);
            super(x,y);
        }

        public Point() {
            this(0, 0, 0);
        }

        public Point(double[] vals) {
            this();
            set(vals);
        }

        public void set(double[] vals) {
            if (vals != null) {
                x = vals.length > 0 ? vals[0] : 0;
                y = vals.length > 1 ? vals[1] : 0;
            } else {
                x = 0;
                y = 0;
            }
        }
    }

    public static float AngleTransform(float angle)
    {
        while (angle>180) angle -= 360;
        while (angle<-180) angle += 360;
        return angle;
    }
    public static float AngleTransformDeg(float angle)
    {
        while (angle>360) angle -= 360;
        while (angle<0) angle += 360;
        return angle;
    }

    public static double angleTransform(double angle)
    {
        while (angle>PI) angle -= PI*2;
        while (angle<-PI) angle += PI*2;
        return angle;
    }
    public static double angleTransformTAU(double angle)
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
