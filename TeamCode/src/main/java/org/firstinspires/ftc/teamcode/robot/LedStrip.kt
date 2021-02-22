package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import java.lang.Math.pow
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.sin

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

    private fun smoothlyLedOn(led: CommandSender, time: Double = 1500.0) {
        val x = if (time != 0.0) 1/time else 1.0
        when {
            ledTime.milliseconds() > time * 3 -> {
                ledTime.reset()
                led.send(0.0)
            }
            ledTime.milliseconds() < time -> {
                led.send(ledTime.milliseconds() * x)
            }
            ledTime.milliseconds() < time * 1.5 -> {
                led.send(1.0)
            }
            ledTime.milliseconds() < time * 2.5 -> {
                led.send(1.0 - (ledTime.milliseconds() - time * 1.5) * x)
            }
            else -> {
                led.send(0.0)
            }
        }
    }
    private fun smoothyLed(led: CommandSender, time: Double = 1500.0, maxPower: Double = 1.0) {
        val x = if (time != 0.0) PI/time else 1.0
        led.send(sin(ledTime.milliseconds() * x).pow(2))
    }
    private fun infromLed() {
        when {
            shooter.rpmNow == 0.0 -> {
                setPowerLed1.send(0.0)
                setPowerLed2.send(0.0)
            }
            shooter.isCorrectRpm() -> {
               setPowerLed1.send(1.0)
               setPowerLed2.send(0.0)
            }
            else -> {
                setPowerLed1.send(0.0)
                setPowerLed2.send(1.0)
            }
        }
    }
    private fun onLed(led: CommandSender, power : Double = 1.0) {
        led.send(power)
    }

    private fun offLed(led: CommandSender) {
        led.send(0.0)
    }
    enum class ledMode {
        SMOOTHLY, ON, OFF, INFORM
    }
}

