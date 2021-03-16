package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.util.ElapsedTime
import java.util.function.DoubleConsumer
import kotlin.math.abs
import kotlin.math.sign

class MotorAccelerationLimiter(private val motorToControl: DoubleConsumer, var maxAcceleration: Double) {
    private val loopTime = ElapsedTime()
    private var currentVelocity = 0.0
    fun setVelocity(requestedVelocity: Double) {
        currentVelocity += abs(requestedVelocity - currentVelocity).coerceAtMost(abs(loopTime.seconds() * maxAcceleration)) * sign(requestedVelocity - currentVelocity)
        if (requestedVelocity == 0.0) motorToControl.accept(0.0) else motorToControl.accept(
            currentVelocity
        )
        loopTime.reset()
    }

    init {
        loopTime.reset()
    }
}