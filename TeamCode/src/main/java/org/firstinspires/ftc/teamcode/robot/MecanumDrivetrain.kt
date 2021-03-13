package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.misc.motorAccelerationLimiter
import org.firstinspires.ftc.teamcode.robot.WoENHardware.controlHubVoltageSensor
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import kotlin.math.abs
import kotlin.math.sign

class MecanumDrivetrain : MultithreadRobotModule(), Drivetrain {
    /* Motor parameters constatnts. */
    @Config
    internal object DrivetrainConfig {
        @JvmField
        var achieveableMaxRPMFraction = 0.885
        @JvmField
        var achieveableMinRPMFraction = 0.095
        @JvmField
        var strafingMultiplier = 1.35
        @JvmField
        var rotationDecrepancy = 1.0
        @JvmField
        var secondsToAccelerate = 0.13
        @JvmField
        var kP = 29.0
        @JvmField
        var kD = 0.0
        @JvmField
        var kI = 0.1
        @JvmField
        var kF = 15.10
        @JvmField
        var kF_referenceVoltage = 13.0
    }

    /* Physical constants */
    private val wheelRadius = 9.8 / 2
    private val gearRatio = 17.0 / 13.0
    private val wheelCenterOffset = Vector2D(18.05253, 15.20000)
    private val forwardMultiplier = (1 / wheelRadius) / gearRatio
    private var sidewaysMultiplier = forwardMultiplier * DrivetrainConfig.strafingMultiplier
    private var turnMultiplier =
        (wheelCenterOffset.x + wheelCenterOffset.y) * DrivetrainConfig.rotationDecrepancy / wheelRadius
    private val tickPerRev = 480.0
    private val gearing = 20.0
    private val maxRPM = 300.0
    private val theoreticalMaxSpeed = maxRPM / 60 * Math.PI * 2
    private var maxMotorVelocity = DrivetrainConfig.achieveableMaxRPMFraction * theoreticalMaxSpeed
    set(value) { field = Range.clip(abs(value), 0.0, theoreticalMaxSpeed)}
    private var minMotorVelocity =
        DrivetrainConfig.achieveableMinRPMFraction * theoreticalMaxSpeed //http://b1-srv-kms-1.sch239.net:8239
    set(value) {field = Range.clip(abs(value), 0.0, theoreticalMaxSpeed)}

    /* Drivetrain hardware members. */
    private lateinit var driveFrontLeft: DcMotorEx
    private lateinit var driveFrontRight: DcMotorEx
    private lateinit var driveRearLeft: DcMotorEx
    private lateinit var driveRearRight: DcMotorEx

    private var maxAcceleration = theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate

    /* Motor controllers */
    private val mFLSender = CommandSender {driveFrontLeft.setVelocity(it, AngleUnit.RADIANS) }
    private val mFLProfiler = motorAccelerationLimiter({mFLSender.send(it)}, maxAcceleration)
    private val mFLDirectSender = CommandSender{driveRearRight.power = it}

    private val mFRSender = CommandSender {driveFrontRight.setVelocity(it, AngleUnit.RADIANS) }
    private val mFRProfiler = motorAccelerationLimiter({mFRSender.send(it) }, maxAcceleration)
    private val mFRDirectSender = CommandSender{driveFrontRight.power = it}

    private val mRLSender = CommandSender {driveRearLeft.setVelocity(it, AngleUnit.RADIANS) }
    private val mRLProfiler = motorAccelerationLimiter({mRLSender.send(it)}, maxAcceleration)
    private val mRLDirectSender = CommandSender{driveRearLeft.power = it}

    private val mRRSender = CommandSender {driveRearRight.setVelocity(it, AngleUnit.RADIANS) }
    private val mRRProfiler = motorAccelerationLimiter({mRRSender.send(it)}, maxAcceleration)
    private val mRRDirectSender = CommandSender{driveRearRight.power = it}

    private lateinit var voltageSensor: VoltageSensor
    private var smartMode = false

    override var targetVelocity = Vector3D(.0,.0,.0)

    private var velocityFrontLeft = 0.0
    private var velocityFrontRight = 0.0
    private var velocityRearLeft = 0.0
    private var velocityRearRight = 0.0
    override fun initialize() {
        assignNames()
        voltageSensor = controlHubVoltageSensor
        setMotorDirections()
        maxMotorVelocity = DrivetrainConfig.achieveableMaxRPMFraction * theoreticalMaxSpeed
        minMotorVelocity = DrivetrainConfig.achieveableMinRPMFraction * theoreticalMaxSpeed
        sidewaysMultiplier = forwardMultiplier * DrivetrainConfig.strafingMultiplier
        maxAcceleration = theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate
        mFLProfiler.maxAcceleration = theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate
        mFRProfiler.maxAcceleration = theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate
        mRLProfiler.maxAcceleration = theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate
        mRRProfiler.maxAcceleration = theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate

        turnMultiplier =
            (wheelCenterOffset.x + wheelCenterOffset.y) * DrivetrainConfig.rotationDecrepancy / wheelRadius
        setMotor0PowerBehaviors(ZeroPowerBehavior.BRAKE)
        setMotorConfiguration(
            DrivetrainConfig.achieveableMaxRPMFraction,
            tickPerRev,
            gearing,
            maxRPM
        )
        try {
            setPIDFCoefficients(
                PIDFCoefficients(
                    DrivetrainConfig.kP,
                    DrivetrainConfig.kD,
                    DrivetrainConfig.kI,
                    DrivetrainConfig.kF * DrivetrainConfig.kF_referenceVoltage / voltageSensor.voltage
                )
            )
        } catch (e: UnsupportedOperationException) {
            opMode.telemetry.addData("Drivetrain PIDF error ", e.message)
        }
        setSmartMode(true)
        targetVelocity = Vector3D(.0,.0,.0)
    }

    fun setSmartMode(SmartMode: Boolean) {
        smartMode = SmartMode
        setMotorMode(if (SmartMode) RunMode.RUN_USING_ENCODER else RunMode.RUN_WITHOUT_ENCODER)
    }

    private fun assignNames() {
        driveFrontLeft = WoENHardware.driveFrontLeft
        driveFrontRight = WoENHardware.driveFrontRight
        driveRearLeft = WoENHardware.driveRearLeft
        driveRearRight = WoENHardware.driveRearRight
    }

    private fun setMotorDirections() {
        driveFrontLeft.direction = DcMotorSimple.Direction.FORWARD
        driveFrontRight.direction = DcMotorSimple.Direction.REVERSE
        driveRearLeft.direction = DcMotorSimple.Direction.FORWARD
        driveRearRight.direction = DcMotorSimple.Direction.REVERSE
    }

    private fun setMotor0PowerBehaviors(zeroPowerBehavior: ZeroPowerBehavior) {
        driveFrontLeft.zeroPowerBehavior = zeroPowerBehavior
        driveFrontRight.zeroPowerBehavior = zeroPowerBehavior
        driveRearLeft.zeroPowerBehavior = zeroPowerBehavior
        driveRearRight.zeroPowerBehavior = zeroPowerBehavior
    }

    private fun setMotorMode(runMode: RunMode) {
        driveFrontLeft.mode = runMode
        driveFrontRight.mode = runMode
        driveRearLeft.mode = runMode
        driveRearRight.mode = runMode
    }

    private fun setPIDFCoefficients(pidfCoefficients: PIDFCoefficients) {
        driveFrontLeft.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveFrontRight.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveRearLeft.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, pidfCoefficients)
        driveRearRight.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, pidfCoefficients)
    }

    private fun setMotorConfiguration(
        achieveableMaxRPMFraction: Double,
        tickPerRev: Double,
        gearing: Double,
        maxRPM: Double
    ) {
        setMotorConfiguration(
            driveFrontLeft,
            achieveableMaxRPMFraction,
            tickPerRev,
            gearing,
            maxRPM
        )
        setMotorConfiguration(
            driveFrontRight,
            achieveableMaxRPMFraction,
            tickPerRev,
            gearing,
            maxRPM
        )
        setMotorConfiguration(driveRearLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
        setMotorConfiguration(
            driveRearRight,
            achieveableMaxRPMFraction,
            tickPerRev,
            gearing,
            maxRPM
        )
    }

    private fun setMotorConfiguration(
        dcMotor: DcMotorEx,
        achieveableMaxRPMFraction: Double,
        tickPerRev: Double,
        gearing: Double,
        maxRPM: Double
    ) {
        val motorConfigurationType = dcMotor.motorType.clone()
        motorConfigurationType.achieveableMaxRPMFraction = achieveableMaxRPMFraction
        motorConfigurationType.ticksPerRev = tickPerRev
        motorConfigurationType.gearing = gearing
        motorConfigurationType.maxRPM = maxRPM
        dcMotor.motorType = motorConfigurationType
    }

    private fun resetEncoders() {
        setMotorMode(RunMode.STOP_AND_RESET_ENCODER)
    }

    override fun updateControlHub() {
        val motorVelocity = Vector3D(targetVelocity.x * sidewaysMultiplier,targetVelocity.y * forwardMultiplier,targetVelocity.z * turnMultiplier)
        velocityFrontLeft = motorVelocity.y + motorVelocity.x + motorVelocity.z
        velocityFrontRight = motorVelocity.y - motorVelocity.x - motorVelocity.z
        velocityRearLeft = motorVelocity.y - motorVelocity.x + motorVelocity.z
        velocityRearRight = motorVelocity.y + motorVelocity.x - motorVelocity.z

        var maxabs = abs(velocityFrontLeft).coerceAtLeast(abs(velocityFrontRight)).coerceAtLeast(abs(velocityRearLeft)).coerceAtLeast(abs(velocityRearRight))
        if (maxabs > maxMotorVelocity) {
            maxabs /= maxMotorVelocity
            velocityFrontLeft /= maxabs
            velocityFrontRight /= maxabs
            velocityRearLeft /= maxabs
            velocityRearRight /= maxabs
        }
        velocityFrontLeft = limitMinSpeed(velocityFrontLeft)
        velocityFrontRight = limitMinSpeed(velocityFrontRight)
        velocityRearLeft = limitMinSpeed(velocityRearLeft)
        velocityRearRight = limitMinSpeed(velocityRearRight)

        if (smartMode) {
            mFLProfiler.setVelocity(velocityFrontLeft)
            mFRProfiler.setVelocity(velocityFrontRight)
            mRLProfiler.setVelocity(velocityRearLeft)
            mRRProfiler.setVelocity(velocityRearRight)
        } else {
            mFLDirectSender.send(velocityFrontLeft / maxMotorVelocity)
            mFRDirectSender.send(velocityFrontRight / maxMotorVelocity)
            mRLDirectSender.send(velocityRearLeft / maxMotorVelocity)
            mRRDirectSender.send(velocityRearRight / maxMotorVelocity)
        }
    }

    override fun updateExpansionHub() {
    }

    override fun updateOther() {
    }


    private fun limitMinSpeed(speed: Double): Double {
        return speed.coerceAtLeast(minMotorVelocity)*sign(speed)
    }

    override val maxVelocity: Vector3D
        get() = Vector3D(
            maxMotorVelocity / forwardMultiplier,
            maxMotorVelocity / forwardMultiplier,
            maxMotorVelocity / turnMultiplier
        )
}