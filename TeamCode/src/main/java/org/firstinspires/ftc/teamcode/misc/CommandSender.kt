package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.util.ElapsedTime
import java.util.function.DoubleConsumer

class CommandSender(private val doubleConsumer: DoubleConsumer, timeout_ms: Double = 3000.0) {
    private val lastCommandTimer = ElapsedTime()
    private var timeout = timeout_ms
    private var lastValue = Double.NaN

   /* constructor(doubleConsumer: DoubleConsumer) {
        this.doubleConsumer = doubleConsumer
    } */

    fun send(value: Double) {
        if (value != lastValue || lastCommandTimer.milliseconds() > timeout) {
            doubleConsumer.accept(value)
            lastValue = value
            lastCommandTimer.reset()
        }
    }
}