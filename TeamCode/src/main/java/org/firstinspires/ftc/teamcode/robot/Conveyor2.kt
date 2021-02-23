package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.misc.motorAccelerationLimiter
import org.firstinspires.ftc.teamcode.robot.WoENHardware.conveyorMotor
import org.firstinspires.ftc.teamcode.robot.WoENHardware.ringDetector
import org.firstinspires.ftc.teamcode.superclasses.Conveyor
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import kotlin.math.abs

class Conveyor2 : MultithreadRobotModule(), Conveyor {
    private lateinit var conveyor: DcMotorEx
    private lateinit var sensorDistance: DistanceSensor
    private val distanceQueryTimeout = 650.0
    private val motorCurrentQueryTimeout = 650.0
    private val conveyorPowerSender = motorAccelerationLimiter({ value: Double ->
        CommandSender { p: Double ->
            conveyor.power = -p
        }.send(value)
    }, 6.0)
    private val motorCurrentTimer = ElapsedTime()
    private val stackDetectionTimer = ElapsedTime()
    private var lastKnownDistance = 12.0
    private val distanceQueryTimer = ElapsedTime()
    private var lastKnownMotorCurrent = 0.0
    private var forceReverse = false
    private var currentMotorPower = 0.0
    private val motorCurrentQueryTimer = ElapsedTime()
    private var doAutomaticConveyorStopping = true
    private var doReverseOnStop = true
    private var requestedPower = 0.0

    @Config
    internal object ConveyorConfig {
        @JvmField
        var conveyorPower = 1.0
        @JvmField
        var motorLockingCurrentTimeout = 400.0
        @JvmField
        var motorLockingReverseTime = 750.0
        @JvmField
        var stackDetectionTimeout = 1000.0
        @JvmField
        var stackDetectionReverseTime = 750.0
        @JvmField
        var distanceThreshold = 5.46
        @JvmField
        var currentThreshold = 2.5
    }

    override fun setAutomaticConveyorStopping(doAutomaticConveyorStopping: Boolean) {
        this.doAutomaticConveyorStopping = doAutomaticConveyorStopping
    }

    override fun setReverseAfterStop(doReverseOnStop: Boolean) {
        this.doReverseOnStop = doReverseOnStop
    }

    override fun setForceReverse(forceReverse: Boolean) {
        this.forceReverse = forceReverse
    }

    override fun enableConveyor(isEnabled: Boolean) {
        setConveyorPower(if (isEnabled) ConveyorConfig.conveyorPower else 0.0)
    }

    private fun setConveyorPower(requestedPower: Double) {
        if (this.requestedPower != requestedPower) motorCurrentTimer.reset()
        this.requestedPower = requestedPower
    }

    override fun initialize() {
        initializecolor()
        initializedrive()
    }

    private fun initializecolor() {
        sensorDistance = ringDetector
    }

    private fun initializedrive() {
        conveyor = conveyorMotor
        conveyor.direction = DcMotorSimple.Direction.FORWARD //!!! can break odometry
        conveyor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }


    private fun getdistance(): Double {
        if (distanceQueryTimer.milliseconds() > distanceQueryTimeout) {
            lastKnownDistance = sensorDistance.getDistance(DistanceUnit.CM)
            distanceQueryTimer.reset()
        }
        return lastKnownDistance
    }

    private val aMPS: Double
        get() {
            if (motorCurrentQueryTimer.milliseconds() > motorCurrentQueryTimeout) {
                lastKnownMotorCurrent = abs(conveyor.getCurrent(CurrentUnit.AMPS))
                motorCurrentQueryTimer.reset()
            }
            return lastKnownMotorCurrent
        }

    override fun updateControlHub() {
        if (!forceReverse && !(doReverseOnStop && requestedPower == 0.0) && doAutomaticConveyorStopping)
            if (getdistance() >= ConveyorConfig.distanceThreshold) {
            stackDetectionTimer.reset() //Full collector detection
        }
    }

    override fun updateExpansionHub() {
        if (!forceReverse) {
            if ((stackDetectionTimer.milliseconds() > ConveyorConfig.stackDetectionTimeout || doReverseOnStop && requestedPower == 0.0) && doAutomaticConveyorStopping) { //reverse+stop in case of ring detection
                motorCurrentTimer.reset()
                currentMotorPower =
                    if (stackDetectionTimer.milliseconds() < ConveyorConfig.stackDetectionTimeout + ConveyorConfig.stackDetectionReverseTime && doReverseOnStop) -1.0 else 0.0
            } else if (motorCurrentTimer.milliseconds() > ConveyorConfig.motorLockingCurrentTimeout) //reverse after locking
            {
                if (motorCurrentTimer.milliseconds() < ConveyorConfig.motorLockingReverseTime + ConveyorConfig.motorLockingCurrentTimeout) {
                    currentMotorPower = -requestedPower
                } else motorCurrentTimer.reset()
            } else {
                currentMotorPower = +requestedPower
                if (aMPS < ConveyorConfig.currentThreshold) //locking detection
                    motorCurrentTimer.reset()
            }
        } else {
            currentMotorPower = -1.0
            motorCurrentTimer.reset()
        }
        conveyorPowerSender.setVelocity(currentMotorPower)
    }

    override fun updateOther() {
    }
}