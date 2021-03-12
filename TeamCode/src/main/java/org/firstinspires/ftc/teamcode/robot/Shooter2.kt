package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.robot.Shooter2.ShooterConfig2.kA
import org.firstinspires.ftc.teamcode.robot.Shooter2.ShooterConfig2.kD
import org.firstinspires.ftc.teamcode.robot.Shooter2.ShooterConfig2.kV_referenceVoltage
import org.firstinspires.ftc.teamcode.robot.Shooter2.ShooterConfig2.kI
import org.firstinspires.ftc.teamcode.robot.Shooter2.ShooterConfig2.kP
import org.firstinspires.ftc.teamcode.robot.Shooter2.ShooterConfig2.kS
import org.firstinspires.ftc.teamcode.robot.Shooter2.ShooterConfig2.kV
import org.firstinspires.ftc.teamcode.robot.Shooter2.ShooterConfig2.maxI
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import org.firstinspires.ftc.teamcode.superclasses.Shooter
import org.openftc.revextensions2.ExpansionHubServo
import kotlin.math.abs
import kotlin.math.sign

class Shooter2: MultithreadRobotModule() {
    private val rpmTime = ElapsedTime()
    private val feederTime = ElapsedTime()
    private val encoderFailureDetectionTime = ElapsedTime()

    @Config
    internal object ShooterConfig2 {
        @JvmField
        var servoTime = 70
        @JvmField
        var servoReturnMultiplier = 3.4
        @JvmField
        var lowRpm = 3470.0
        @JvmField
        var highRpm = 4000.0
      //  @JvmField
      //  var timeRpm = 150.0
        @JvmField
        var feederClose = 0.23
        @JvmField
        var feederOpen = 0.49
        @JvmField
        var kP = 186.0
        @JvmField
        var kI = 1.77 //0.03
        @JvmField
        var kD = 0.0
        @JvmField
        var kV = 13.56
        @JvmField
        var kA = 1.0
        @JvmField
        var kS = 3000.0
        @JvmField
        var maxI = 600000.0
    //    @JvmField
      //  var maxRPM = 5400.0
      //  @JvmField
      //  var kF = 14.89
        @JvmField
        var kV_referenceVoltage = 12.485
    }

    private lateinit var shooterMotor: DcMotorEx
    private lateinit var voltageSensor: VoltageSensor
    private lateinit var feeder: ExpansionHubServo
    private val shooterPowerSender = CommandSender { p: Double -> shooterMotor.power = p }
    private val feederPositionSender = CommandSender { p: Double -> feeder.position = p }
    private var shooterMode = Shooter.ShooterMode.OFF
    private var ringsToShoot: Int = 0
    private var timeToAccelerateMs = 1.0
    private var accelerationIncrement = 1.0
    var rpmTarget = 6000.0
        private set
    private var motorVelocityTarget = 0.0
    var currentRpm = 0.0
    var encoderFailureMode = false
        private set

    override fun initialize() {
        voltageSensor = WoENHardware.expansionHubVoltageSensor
        shooterMotor = WoENHardware.shooterMotor
        /*val motorConfigurationType = shooterMotor.motorType.clone()
        motorConfigurationType.achieveableMaxRPMFraction = 0.896476253
        motorConfigurationType.ticksPerRev = 24.0
        motorConfigurationType.gearing = 1.0
        motorConfigurationType.maxRPM = 6000.0
        shooterMotor.motorType = motorConfigurationType */
        shooterMotor.direction = DcMotorSimple.Direction.FORWARD
        shooterMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        shooterMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shootingMode = Shooter.ShooterMode.OFF
        initializedservo()
        feederTime.reset()
        rpmTime.reset()
    }

    private fun initializedservo() {
        feeder = opMode.hardwareMap.get(Servo::class.java, "feeder") as ExpansionHubServo
        feeder.position = ShooterConfig2.feederClose
    }

    override fun start() {
        feeder.position = ShooterConfig2.feederClose
        shooterMotor.power = 0.0
        shootingMode = Shooter.ShooterMode.OFF
        ringsToShoot = 0
    }

    override fun updateControlHub() {
        if (ringsToShoot > 0 && feederTime.milliseconds() > ShooterConfig2.servoTime * ShooterConfig2.servoReturnMultiplier) {
            feedRing()
            ringsToShoot--
        }
        setFeederPosition(feederTime.milliseconds() < ShooterConfig2.servoTime && rpmTarget != 0.0)
    }
    private var velocityError = 0.0
    private var velocityErrorOld = 0.0
    private var P = 0.0
    private var D = 0.0
    private var I = 0.0
    private var V = 0.0
    private var A = 0.0
    private var S = 0.0
    private var power = 0.0
    private var timeOld = 0.0
    private var timeDelta = 0.0
    private var voltageDelta = 0.0
    private var velocityTargetOld = 0.0
    private var currentVelocity = 0.0
    override fun updateExpansionHub() {
        timeDelta = rpmTime.seconds() - timeOld
        timeOld = rpmTime.seconds()
        currentVelocity = getMotorVelocity()
        currentRpm = currentVelocity * 2.5
        voltageDelta = kV_referenceVoltage / voltageSensor.voltage
        if (motorVelocityTarget != 0.0) {
            velocityError = motorVelocityTarget - currentVelocity
            P = velocityError * kP
            D = (velocityError - velocityErrorOld) * kD / timeDelta
            I += (kI * velocityError) * timeDelta
            if (abs(I) > maxI) I = sign(I) * maxI
            V = kV * motorVelocityTarget * voltageDelta
            A = kA * (motorVelocityTarget - velocityTargetOld) / timeDelta * voltageDelta
            S = kS * sign(motorVelocityTarget) * voltageDelta
            power = (P + I + D + V + A + S) / 32768
            velocityErrorOld = velocityError
            velocityTargetOld = motorVelocityTarget
        }
        else {
            velocityError = 0.0
            power = 0.0
            velocityErrorOld = 0.0
            I = 0.0
            velocityTargetOld = 0.0
        }
        shooterPowerSender.send(power)
    }

    override fun updateOther() {
    }

    private fun setFeederPosition(push: Boolean) {
        feederPositionSender.send(if (push) ShooterConfig2.feederOpen else ShooterConfig2.feederClose)
    }

    private fun setShootersetings(Rpm: Double) {
        if (Rpm != rpmTarget) {
            rpmTarget = Rpm
            motorVelocityTarget = rpmTarget * 0.4
        }
    }

    private fun getMotorVelocity(): Double = shooterMotor.velocity //* 2.5

    var shootingMode: Shooter.ShooterMode
        get() = shooterMode
        set(mode) {
            //if (mode != Shooter.ShooterMode.OFF && shooterMode == Shooter.ShooterMode.OFF) rpmTime.reset()
            shooterMode = mode
            when (mode) {
                Shooter.ShooterMode.HIGHGOAL -> setShootersetings(
                    ShooterConfig2 .highRpm
                )
                Shooter.ShooterMode.POWERSHOT -> setShootersetings(
                    ShooterConfig2.lowRpm
                )
                Shooter.ShooterMode.OFF -> setShootersetings(0.0)
            }
        }

    fun isCorrectRpm(error: Double = 30.0): Boolean {
        return if (encoderFailureMode) true else abs(rpmTarget - currentRpm) < error // abs(currentVelocity - rpmNow / 2.5) < error
    }

    fun feedRing() {
        //  ringsToShoot = 1
        feederTime.reset()
    }

    fun feedRings() {
        ringsToShoot = 3
    }
}