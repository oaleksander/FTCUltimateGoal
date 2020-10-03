package org.firstinspires.ftc.teamcode.math;

import org.jetbrains.annotations.NotNull;

public class Vector2D implements Comparable {

    public double x = 0;
    public double y = 0;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D rotated(double angle) {
        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Vector2D(newX, newY);
    }

    public Vector2D add(@NotNull Vector2D p) {
        return new Vector2D(this.x + p.x, this.y + p.y);
    }
    public Vector2D minus(@NotNull Vector2D p) {
        return new Vector2D(this.x - p.x, this.y - p.y);
    }

    public double atan() {
        return Math.atan2(y, x);
    }

    public double radius() {
        return Math.sqrt(x * x + y * y);
    }
    public double distance(Vector2D p) {return minus(p).radius(); }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Vector2D vector2D = (Vector2D) o;
        return MathUtil.approxEquals(vector2D.x, x) &&
                MathUtil.approxEquals(vector2D.y, y);
    }

    @Override
    public int hashCode() {
        return Double.valueOf(x).hashCode() ^ Double.valueOf(y).hashCode();
    }

    @Override
    public String toString() {
        return String.format("(%.1f, %.1f)", x, y);
    }

    @Override
    public int compareTo(Object o) {
        if (this == o) return 0;
        if (o == null || getClass() != o.getClass()) return -1;
        Vector2D p = (Vector2D) o;
        return Integer.compare(this.hashCode(), o.hashCode());
    }

}