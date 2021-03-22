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
import org.firstinspires.ftc.teamcode.misc.MotorAccelerationLimiter
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.VelocityOdometry
import org.firstinspires.ftc.teamcode.superclasses.VoltageSupplier
import kotlin.math.abs
import kotlin.math.sign

class MecanumDrivetrain(private val voltageSupplier: VoltageSupplier) : MultithreadedRobotModule(), Drivetrain, VelocityOdometry {
    /* Motor parameters constatnts. */
    @Config
    internal object DrivetrainConfig {
        @JvmField var achieveableMaxRPMFraction = 0.885
        @JvmField var achieveableMinRPMFraction = 0.045
        @JvmField var strafingMultiplier = 1.25
        @JvmField var rotationDecrepancy = 1.0
        @JvmField var secondsToAccelerate = 0.10
        @JvmField var kP = 45.0
        @JvmField var kD = 0.0
        @JvmField var kI = 0.05
        @JvmField var kF = 15.10
        @JvmField var kF_referenceVoltage = 13.0
    }

    /* Physical constants */
    private val wheelRadius = 9.8 / 2
    private val gearRatio = 17.0 / 13.0
    private val wheelCenterOffset = Vector2D(18.05253, 15.20000)
    private val forwardMultiplier = (1 / wheelRadius) / gearRatio
    private var sidewaysMultiplier = forwardMultiplier * DrivetrainConfig.strafingMultiplier
    private var turnMultiplier = (wheelCenterOffset.x + wheelCenterOffset.y) * DrivetrainConfig.rotationDecrepancy / wheelRadius
    private val tickPerRev = 480.0
    private val gearing = 20.0
    private val maxRPM = 300.0
    private val theoreticalMaxSpeed = maxRPM / 60 * Math.PI * 2
    private var maxMotorVelocity = DrivetrainConfig.achieveableMaxRPMFraction * theoreticalMaxSpeed
        set(value) {
            field = Range.clip(abs(value), 0.0, theoreticalMaxSpeed)
        }
    private var minMotorVelocity = DrivetrainConfig.achieveableMinRPMFraction * theoreticalMaxSpeed //http://b1-srv-kms-1.sch239.net:8239
        set(value) {
            field = Range.clip(abs(value), 0.0, theoreticalMaxSpeed)
        }

    /* Drivetrain hardware members. */
    private lateinit var driveFrontLeft: DcMotorEx
    private lateinit var driveFrontRight: DcMotorEx
    private lateinit var driveRearLeft: DcMotorEx
    private lateinit var driveRearRight: DcMotorEx

    private var maxAcceleration = theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate

    /* Motor controllers */
    private val mFLSender = CommandSender({ driveFrontLeft.setVelocity(it, AngleUnit.RADIANS) })
    private val mFLProfiler = MotorAccelerationLimiter({ mFLSender.send(it) }, {theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate})
    private val mFLDirectSender = CommandSender({ driveRearRight.power = it })

    private val mFRSender = CommandSender({ driveFrontRight.setVelocity(it, AngleUnit.RADIANS) })
    private val mFRProfiler = MotorAccelerationLimiter({ mFRSender.send(it) }, {theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate})
    private val mFRDirectSender = CommandSender({ driveFrontRight.power = it })

    private val mRLSender = CommandSender({ driveRearLeft.setVelocity(it, AngleUnit.RADIANS) })
    private val mRLProfiler = MotorAccelerationLimiter({ mRLSender.send(it) }, {theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate})
    private val mRLDirectSender = CommandSender({ driveRearLeft.power = it })

    private val mRRSender = CommandSender({ driveRearRight.setVelocity(it, AngleUnit.RADIANS) })
    private val mRRProfiler = MotorAccelerationLimiter({ mRRSender.send(it) }, {theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate})
    private val mRRDirectSender = CommandSender({ driveRearRight.power = it })

    private lateinit var voltageSensor: VoltageSensor
    private var smartMode = false

    override var targetVelocity = Vector3D(.0, .0, .0)

    private var targetVelocityFL = 0.0
    private var targetVelocityFR = 0.0
    private var targetVelocityRL = 0.0
    private var targetVelocityRR = 0.0
    private var measuredVelocityFL = 0.0
    private var measuredVelocityFR = 0.0
    private var measuredVelocityRL = 0.0
    private var measuredVelocityRR = 0.0
    override fun initialize() {
        assignNames()
        setMotorDirections()
        maxMotorVelocity = DrivetrainConfig.achieveableMaxRPMFraction * theoreticalMaxSpeed
        minMotorVelocity = DrivetrainConfig.achieveableMinRPMFraction * theoreticalMaxSpeed
        sidewaysMultiplier = forwardMultiplier * DrivetrainConfig.strafingMultiplier

        turnMultiplier = (wheelCenterOffset.x + wheelCenterOffset.y) * DrivetrainConfig.rotationDecrepancy / wheelRadius
        setMotorConfiguration(DrivetrainConfig.achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
        try {
            setPIDFCoefficients(PIDFCoefficients(DrivetrainConfig.kP, DrivetrainConfig.kD, DrivetrainConfig.kI,
                                                 DrivetrainConfig.kF * DrivetrainConfig.kF_referenceVoltage / voltageSupplier.voltage))
        } catch (e: UnsupportedOperationException) {
            opMode.telemetry.addData("Drivetrain PIDF error ", e.message)
        }
        setMotor0PowerBehaviors(ZeroPowerBehavior.BRAKE)
        setSmartMode(true)
        targetVelocity = Vector3D(.0, .0, .0)
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

    private fun setMotorConfiguration(achieveableMaxRPMFraction: Double, tickPerRev: Double, gearing: Double, maxRPM: Double) {
        setMotorConfiguration(driveFrontLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
        setMotorConfiguration(driveFrontRight, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
        setMotorConfiguration(driveRearLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
        setMotorConfiguration(driveRearRight, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM)
    }

    private fun setMotorConfiguration(dcMotor: DcMotorEx, achieveableMaxRPMFraction: Double, tickPerRev: Double, gearing: Double, maxRPM: Double) {
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
        measuredVelocityFL = driveFrontLeft.velocity
        measuredVelocityFR = driveFrontRight.velocity
        measuredVelocityRL = driveRearLeft.velocity
        measuredVelocityRR = driveRearRight.velocity

        robotVelocity = Vector3D((measuredVelocityFL - measuredVelocityFR - measuredVelocityRL - measuredVelocityRR) / (4 * sidewaysMultiplier),
                                 (measuredVelocityFL + measuredVelocityFR + measuredVelocityRL + measuredVelocityRR) / (4 * forwardMultiplier),
                                 (measuredVelocityFL - measuredVelocityFR + measuredVelocityRL - measuredVelocityRR) / (4 * turnMultiplier))

        val targetChassisVelocityTicks = Vector3D(targetVelocity.x * sidewaysMultiplier, targetVelocity.y * forwardMultiplier,
                                                  targetVelocity.z * turnMultiplier)
        targetVelocityFL = targetChassisVelocityTicks.y + targetChassisVelocityTicks.x + targetChassisVelocityTicks.z
        targetVelocityFR = targetChassisVelocityTicks.y - targetChassisVelocityTicks.x - targetChassisVelocityTicks.z
        targetVelocityRL = targetChassisVelocityTicks.y - targetChassisVelocityTicks.x + targetChassisVelocityTicks.z
        targetVelocityRR = targetChassisVelocityTicks.y + targetChassisVelocityTicks.x - targetChassisVelocityTicks.z

        var maxabs = abs(targetVelocityFL).coerceAtLeast(abs(targetVelocityFR)).coerceAtLeast(abs(targetVelocityRL))
            .coerceAtLeast(abs(targetVelocityRR))
        if (maxabs > maxMotorVelocity) {
            maxabs /= maxMotorVelocity
            targetVelocityFL /= maxabs
            targetVelocityFR /= maxabs
            targetVelocityRL /= maxabs
            targetVelocityRR /= maxabs
        }
        targetVelocityFL = limitMinSpeed(targetVelocityFL)
        targetVelocityFR = limitMinSpeed(targetVelocityFR)
        targetVelocityRL = limitMinSpeed(targetVelocityRL)
        targetVelocityRR = limitMinSpeed(targetVelocityRR)

        if (smartMode) {
            mFLProfiler.setVelocity(targetVelocityFL)
            mFRProfiler.setVelocity(targetVelocityFR)
            mRLProfiler.setVelocity(targetVelocityRL)
            mRRProfiler.setVelocity(targetVelocityRR)
        } else {
            mFLDirectSender.send(targetVelocityFL / maxMotorVelocity)
            mFRDirectSender.send(targetVelocityFR / maxMotorVelocity)
            mRLDirectSender.send(targetVelocityRL / maxMotorVelocity)
            mRRDirectSender.send(targetVelocityRR / maxMotorVelocity)
        }
    }

    override fun updateExpansionHub() {
    }

    override fun updateOther() {
    }


    private fun limitMinSpeed(speed: Double): Double {
        return abs(speed).coerceAtLeast(minMotorVelocity) * sign(speed)
    }

    override val maxVelocity: Vector3D
        get() = Vector3D(maxMotorVelocity / forwardMultiplier, maxMotorVelocity / forwardMultiplier, maxMotorVelocity / turnMultiplier)
    override var robotVelocity: Vector3D = Vector3D()
}