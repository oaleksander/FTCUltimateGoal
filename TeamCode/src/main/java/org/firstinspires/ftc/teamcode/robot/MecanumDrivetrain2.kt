package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.misc.MotorAccelerationLimiter
import org.firstinspires.ftc.teamcode.misc.RegulatorPIDVAS
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain2.DrivetrainConfig.kA
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain2.DrivetrainConfig.kD
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain2.DrivetrainConfig.kI
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain2.DrivetrainConfig.kP
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain2.DrivetrainConfig.kS
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain2.DrivetrainConfig.kV
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain2.DrivetrainConfig.kV_referenceVoltage
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain2.DrivetrainConfig.maxI
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.VelocityOdometry
import org.firstinspires.ftc.teamcode.superclasses.VoltageSupplier
import kotlin.math.abs
import kotlin.math.sign

class MecanumDrivetrain2 (private val voltageSupplier: VoltageSupplier) : MultithreadedRobotModule(), Drivetrain, VelocityOdometry {
    /* Motor parameters constatnts. */
    @Config
    @Disabled
    internal object DrivetrainConfig {
        @JvmField var achieveableMaxRPMFraction = 0.885
        @JvmField var achieveableMinRPMFraction = 0.045
        @JvmField var strafingMultiplier = 1.25
        @JvmField var rotationDecrepancy = 1.0
        @JvmField var secondsToAccelerate = 0.10
        @JvmField var kP = 45.0
        @JvmField var kD = 0.0
        @JvmField var kI = 0.05
        @JvmField var kV = 15.10
        @JvmField var kA = 0.0
        @JvmField var kS = 0.0
        @JvmField var maxI = 60000000000.0
        @JvmField var kV_referenceVoltage = 13.0
    }

    /* Physical constants */
    private val wheelRadius = 9.8 / 2
    private val gearRatio = 17.0 / 13.0
    private val wheelCenterOffset = Vector2D(18.05253, 15.20000)
    private val forwardMultiplier = (1 / wheelRadius) / gearRatio
    private var sidewaysMultiplier = forwardMultiplier * DrivetrainConfig.strafingMultiplier
    private var turnMultiplier = (wheelCenterOffset.x + wheelCenterOffset.y) * DrivetrainConfig.rotationDecrepancy / wheelRadius
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

    /* Motor controllers */
    private val mFLSender = CommandSender({ driveFrontLeft.power = it })
    private val mFLReg = RegulatorPIDVAS({ mFLSender.send(it)},{measuredVelocityFL}, {voltageSupplier.voltage}, {kP}, {kD}, {kI}, {kV}, {kA}, {kS}, {maxI}, {kV_referenceVoltage})

    private val mFRSender = CommandSender({ driveFrontRight.power = it })
    private val mFRReg = RegulatorPIDVAS({ mFRSender.send(it)},{measuredVelocityFR}, {voltageSupplier.voltage}, {kP}, {kD}, {kI}, {kV}, {kA}, {kS}, {maxI}, {kV_referenceVoltage})

    private val mRLSender = CommandSender({ driveRearLeft.power = it })
    private val mRLReg = RegulatorPIDVAS({ mRLSender.send(it)},{measuredVelocityRL}, {voltageSupplier.voltage}, {kP}, {kD}, {kI}, {kV}, {kA}, {kS}, {maxI}, {kV_referenceVoltage})

    private val mRRSender = CommandSender({ driveRearRight.power = it })
    private val mRRReg = RegulatorPIDVAS({ mRRSender.send(it)},{measuredVelocityRR}, {voltageSupplier.voltage}, {kP}, {kD}, {kI}, {kV}, {kA}, {kS}, {maxI}, {kV_referenceVoltage})

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
        resetEncoders()
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        maxMotorVelocity = DrivetrainConfig.achieveableMaxRPMFraction * theoreticalMaxSpeed
        minMotorVelocity = DrivetrainConfig.achieveableMinRPMFraction * theoreticalMaxSpeed
        sidewaysMultiplier = forwardMultiplier * DrivetrainConfig.strafingMultiplier
        turnMultiplier = (wheelCenterOffset.x + wheelCenterOffset.y) * DrivetrainConfig.rotationDecrepancy / wheelRadius
        setMotor0PowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE)
        //setSmartMode(true)
        targetVelocity = Vector3D(.0, .0, .0)
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

    private fun setMotor0PowerBehaviors(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        driveFrontLeft.zeroPowerBehavior = zeroPowerBehavior
        driveFrontRight.zeroPowerBehavior = zeroPowerBehavior
        driveRearLeft.zeroPowerBehavior = zeroPowerBehavior
        driveRearRight.zeroPowerBehavior = zeroPowerBehavior
    }

    private fun setMotorMode(runMode: DcMotor.RunMode) {
        driveFrontLeft.mode = runMode
        driveFrontRight.mode = runMode
        driveRearLeft.mode = runMode
        driveRearRight.mode = runMode
    }

    private fun resetEncoders() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
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

        mFLReg.updateRegulator(targetVelocityFL)
        mFRReg.updateRegulator(targetVelocityFR)
        mRLReg.updateRegulator(targetVelocityRL)
        mRRReg.updateRegulator(targetVelocityRR)
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