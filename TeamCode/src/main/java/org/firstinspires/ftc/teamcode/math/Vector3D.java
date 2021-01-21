package org.firstinspires.ftc.teamcode.math;

import java.util.Locale;

public class Vector3D extends Vector2D implements Cloneable {
    public double z;

    public Vector3D(double x, double y, double z) {
        super(x, y);
        this.z = z;
    }

    public Vector3D(Pose2D p) {
        this(p.x, p.y, p.heading);
    }

    public Vector3D() {
        this(0, 0, 0);
    }

    public Vector3D(Vector2D p, double z) {
        this(p.x, p.y, z);
    }

    public Vector3D add(Vector3D p2) {
        return new Vector3D(x + p2.x, y + p2.y, z + p2.z);
    }

    public Vector3D substract(Vector3D p2) {
        return new Vector3D(x - p2.x, y - p2.y, z - p2.z);
    }

    public Vector3D multiply(Vector3D p2) {
        return new Vector3D(x * p2.x, y * p2.y, z * p2.z);
    }

    public Vector3D divideByDouble(double d) {
        return new Vector3D(x / d, y / d, z / d);
    }

    public Vector3D minus(Vector3D p2) {
        return new Vector3D(x - p2.x, y - p2.y, z - p2.z);
    }

    public Vector3D scale(double d) {
        return new Vector3D(x * d, y * d, z * d);
    }

/*    public void normalize() {
        double r = radius();
        if (radius() != 0) {
            x /= radius();
            y /= radius();
            z /= radius();
        }
    }*/

    public Vector3D normalize() {
        double r = radius();
        if (radius() != 0) {
            return new Vector3D(x /= radius(), y /= radius(), z /= radius());
        }
        return this;
    }

    public double radius() {
        return Math.sqrt(x * x + y * y + z * z);
    }


    public void clampAbs(Vector3D p2) {
        x = Math.copySign(minAbs(x, p2.x), x);
        y = Math.copySign(minAbs(y, p2.y), y);
        z = Math.copySign(minAbs(z, p2.z), z);
    }

    public void applyFriction(Vector3D friction) {
        x = reduceUpToZero(x, friction.x);
        y = reduceUpToZero(y, friction.y);
        z = reduceUpToZero(z, friction.z);
    }

    private double reduceUpToZero(double d, double reduction) {
        return d - minAbs(d, Math.copySign(reduction, d));
    }

    private double minAbs(double a, double b) {
        return Math.abs(a) < Math.abs(b) ? a : b;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;
        Vector3D Vector3D = (Vector3D) o;
        return MathUtil.approxEquals(Vector3D.z, z);
    }

    @Override
    public String toString() {
        return String.format(Locale.getDefault(), "{x: %.3f, y: %.3f, z: %.3f}", x, y, z);
    }

    @Override
    public Vector3D clone() {
        return new Vector3D(x, y, z);
    }
}
