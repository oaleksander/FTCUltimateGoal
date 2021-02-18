package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.robot.WoENHardware.conveyorMotor
import org.firstinspires.ftc.teamcode.robot.WoENHardware.ringDetector
import org.firstinspires.ftc.teamcode.superclasses.Conveyor
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import org.firstinspires.ftc.teamcode.superclasses.RobotModule

@Deprecated("")
class Conveyor1 : MultithreadRobotModule(), Conveyor {
    private val conveyorTime = ElapsedTime()
    private val backOnTime = ElapsedTime()
    private val pauseTime = ElapsedTime()
    private val BackOnAftertime = ElapsedTime()
    private val conveyorPowerSender = CommandSender { p: Double -> conveyorm.power = -p }
    private var full = false
    private var backOn = false
    private var stop = false
    private var backMust = false
    private var backOnAfter = false
    private var colorLock = false
    private var timelock = 0.0
    private var conveyorPower = 0.0
    private var distance = 0.0
    private var current = 0.0
    override fun initialize() {
        initializecolor()
        initializedrive()
    }

    override fun start() {
        conveyorm.power = 0.0
        backOn = false
        stop = false
        conveyorPower = 0.0
    }

    private fun initializecolor() {
        sensorDistance = ringDetector
    }

    private fun initializedrive() {
        conveyorm = conveyorMotor
        conveyorm.direction = DcMotorSimple.Direction.FORWARD
        conveyorm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private fun getdistance(): Double {
        //return 10;
        return sensorDistance.getDistance(DistanceUnit.CM)
    }

    override fun updateControlHub() {
        if (pauseTime.milliseconds() >= 100) {
            pauseTime.reset()
            distance = if (!colorLock) getdistance() else 10.0
        }
    }

    override fun updateExpansionHub() {
        current = conveyorm.getCurrent(CurrentUnit.AMPS)
        if (distance < 6) {
            if (conveyorTime.milliseconds() >= 1000) {
                full = true
            }
        } else {
            conveyorTime.reset()
            full = false
        }
        if (!backMust) {
            if (conveyorPower != 0.0 && !full) {
                if (!stop) {
                    stop = true
                }
                if (current <= 4 && backOnTime.milliseconds() >= 1000) {
                    if (!backOn) {
                        setConveyorMotorPower(conveyorPower)
                        backOn = true
                    }
                    timelock = backOnTime.milliseconds()
                    BackOnAftertime.reset()
                } else {
                    if (backOn && backOnTime.milliseconds() >= timelock + 500) {
                        backOnTime.reset()
                        setConveyorMotorPower(-conveyorPower)
                        backOn = false
                    }
                }
            } else {
                if (stop) {
                    if (backOnAfter && BackOnAftertime.milliseconds() < 500) setConveyorMotorPower(-conveyorPower) else {
                        setConveyorMotorPower(0.0)
                        stop = false
                        backOn = false
                    }
                }
            }
        } else {
            setConveyorMotorPower(-1.0)
            backOn = false
            stop = true
        }
    }

    override fun setReverseAfterStop(BackOnAfter: Boolean) {
        backOnAfter = BackOnAfter
    }

    override fun setConveyorPower(power: Double) {
        conveyorPower = power
    }

    override fun setAutomaticConveyorStopping(doAutomaticConveyorStopping: Boolean) {
        colorLock = !doAutomaticConveyorStopping
    }

    private fun setConveyorMotorPower(power: Double) {
        conveyorPowerSender.send(power)
    }

    override fun setForceReverse(Backmust: Boolean) {
        backMust = Backmust
    }

    companion object {
        private lateinit var conveyorm: DcMotorEx
        private lateinit var sensorDistance: DistanceSensor
    }
}