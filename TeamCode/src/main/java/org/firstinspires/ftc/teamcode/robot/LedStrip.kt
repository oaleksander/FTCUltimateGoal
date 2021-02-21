package org.firstinspires.ftc.teamcode.robot

import androidx.renderscript.ScriptIntrinsicBLAS
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
@Deprecated("")
class LedStrip: MultithreadRobotModule() {
    private lateinit var ledStrip1 : DcMotorEx
    private lateinit var ledStrip2 : DcMotorEx
    private val ledTime = ElapsedTime()

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
    private fun smoothlyLedOn(led : DcMotorEx, time : Double = 1000.0) {
        ledTime.reset()
    }
}