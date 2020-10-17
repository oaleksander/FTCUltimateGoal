package org.firstinspires.ftc.teamcode.math;

import org.jetbrains.annotations.NotNull;

import static org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap;

public class Pose2D extends Vector2D implements Cloneable {
    public double heading = 0;

    public Pose2D(double x, double y, double heading) {
        super(x, y);
        this.heading = angleWrap(heading);
    }
    public Pose2D() {
        this(0,0,0);
    }

    public Pose2D(@NotNull Vector2D p, double heading) {
        this(p.x, p.y, angleWrap(heading));
    }

    public Pose2D add(@NotNull Pose2D p2) {
        return new Pose2D(x + p2.x, y + p2.y, heading + p2.heading);
    }
    public Pose2D substract(@NotNull Pose2D p2) {
        return new Pose2D(x - p2.x, y - p2.y, heading - p2.heading);
    }
    public Pose2D multiply(@NotNull Pose2D p2) {
        return new Pose2D(x * p2.x, y * p2.y, heading * p2.heading);
    }
    public Pose2D divideByPose(@NotNull Pose2D p2) {
        return new Pose2D(x / p2.x, y / p2.y, heading / p2.heading);
    }
    public Pose2D divideByDouble(@NotNull double d) {
        return new Pose2D(x / d, y / d, heading / d);
    }
    public Pose2D minus(@org.jetbrains.annotations.NotNull Pose2D p2) {
        return new Pose2D(x - p2.x, y - p2.y, heading - p2.heading);
    }
    public Pose2D scale(double d) {return new Pose2D(x * d, y * d, heading * d);}
    public void clampAbs(@NotNull Pose2D p2) {
        x = Math.copySign(minAbs(x, p2.x), x);
        y = Math.copySign(minAbs(y, p2.y), y);
        heading = Math.copySign(minAbs(heading, p2.heading), heading);
    }

    public void applyFriction(@NotNull Pose2D friction) {
        x = reduceUpToZero(x, friction.x);
        y = reduceUpToZero(y, friction.y);
        heading = reduceUpToZero(heading, friction.heading);
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
        Pose2D pose2D = (Pose2D) o;
        return MathUtil.approxEquals(pose2D.heading, heading);
    }

    @Override
    public String toString() {
        return String.format("{x: %.3f, y: %.3f, θ: %.3f}", x, y, heading);
    }

    @Override
    public Pose2D clone() {
        return new Pose2D(x, y, heading);
    }
}