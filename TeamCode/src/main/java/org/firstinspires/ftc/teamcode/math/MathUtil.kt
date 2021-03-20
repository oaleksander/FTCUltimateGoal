package org.firstinspires.ftc.teamcode.math

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sqrt

object MathUtil {
    const val EPSILON = 1e-6
    @JvmStatic
    @JvmOverloads
    fun approxEquals(d1: Double, d2: Double, minDelta: Double = EPSILON): Boolean {
        return abs(d1 - d2) < minDelta
    }

    @JvmStatic
    fun angleWrap(Angle: Double): Double {
        var angle = Angle
        while (angle > PI) angle -= PI * 2
        while (angle < -PI) angle += PI * 2
        return angle
    }

    fun angleWrapHalf(Angle: Double): Double {
        var angle = Angle
        while (angle > PI / 2) angle -= PI
        while (angle < -PI / 2) angle += PI
        return angle
    }

    @JvmStatic
    fun angleAverage(angle1: Double, angle2: Double): Double {
        return angleWrap(angle1 + angleWrap(angle2 - angle1) / 2)
    }

    fun minAbs(a: Double, b: Double): Double {
        return if (abs(a) < abs(b)) a else b
    }

    @JvmStatic
    fun cosFromSin(sin: Double, Angle: Double): Double {
        var angle = Angle
        var cos = sqrt(1 - sin * sin)
        angle = angleWrap(angle)
        if (angle > PI / 2 || angle < -PI / 2) cos *= -1.0
        return cos
    }
}