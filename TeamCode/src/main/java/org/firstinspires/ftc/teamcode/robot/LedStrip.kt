package org.firstinspires.ftc.teamcode.robot

import androidx.renderscript.ScriptIntrinsicBLAS
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
@Deprecated("")
class LedStrip: MultithreadRobotModule() {
    private lateinit var ledStrip1 : DcMotorEx
    private lateinit var ledStrip2 : DcMotorEx
    private val ledTime = ElapsedTime()
    private val setPowerLed1 = CommandSender {p : Double -> ledStrip1.power = p}
    private val setPowerLed2 = CommandSender {p : Double -> ledStrip2.power = p}


    override fun initialize() {
        ledStrip1 = WoENHardware.ledStrip1
        ledStrip2 = WoENHardware.ledStrip2
        ledTime.reset()
    }
    override fun updateControlHub() {

    }

    override fun updateExpansionHub() {

    }

    override fun updateOther() {

    }
    private fun smoothlyLedOn(led : DcMotorEx, time :Double = 1500.0) {
        val x = if (time != 0.0) 1/time else 1.0
        when {
            ledTime.milliseconds() > time * 3 -> {
                ledTime.reset()
                //led.power = 0.0
            }
            ledTime.milliseconds() < time -> {
                //led.power = ledTime.milliseconds() * x
            }
            ledTime.milliseconds() < time * 1.5 -> {
                //led.power = 1.0
            }
            ledTime.milliseconds() < time * 2.5 -> {
                //led.power = 1.0 - (ledTime.milliseconds() - time * 1.5) * x
            }
            else -> {
                //led.power = 0.0
            }
        }
    }
}