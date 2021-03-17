package org.firstinspires.ftc.teamcode.math

import kotlin.math.min

object MathUtil {
    const val EPSILON = 1e-6
    @JvmStatic
    @JvmOverloads
    fun approxEquals(d1: Double, d2: Double, minDelta: Double = EPSILON): Boolean {
        return Math.abs(d1 - d2) < minDelta
    }

    @JvmStatic
    fun angleWrap(angle: Double): Double {
        var angle = angle
        while (angle > Math.PI) angle -= Math.PI * 2
        while (angle < -Math.PI) angle += Math.PI * 2
        return angle
    }

    fun angleWrapHalf(angle: Double): Double {
        var angle = angle
        while (angle > Math.PI / 2) angle -= Math.PI
        while (angle < -Math.PI / 2) angle += Math.PI
        return angle
    }

    @JvmStatic
    fun angleAverage(angle1: Double, angle2: Double): Double {
        return angleWrap(angle1 + angleWrap(angle2 - angle1) / 2)
    }

    fun minAbs(a: Double, b: Double): Double {
        return if (Math.abs(a) < Math.abs(b)) a else b
    }

    @JvmStatic
    fun cosFromSin(sin: Double, angle: Double): Double {
        var angle = angle
        var cos = Math.sqrt(1 - sin * sin)
        angle = angleWrap(angle)
        if (angle > Math.PI / 2 || angle < -Math.PI / 2) cos *= -1.0
        return cos
    }
}