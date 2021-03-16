package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.math.MathUtil
import java.util.function.DoubleConsumer

class CommandSender(private val doubleConsumer: DoubleConsumer, timeout_ms: Double = 3000.0) {
    private val lastCommandTimer = ElapsedTime()
    private var timeout = timeout_ms
    private var lastValue = Double.NaN

    fun send(value: Double) { //Math.abs(value-lastValue) < 1/65536.0 ?
        if (!MathUtil.approxEquals(value, lastValue) || lastCommandTimer.milliseconds() > timeout) {
            doubleConsumer.accept(value)
            lastValue = value
            lastCommandTimer.reset()
        }
    }
}