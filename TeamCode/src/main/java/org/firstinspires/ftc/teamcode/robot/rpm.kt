package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.lynx.LynxVoltageSensor
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import org.openftc.revextensions2.ExpansionHubServo

class rpm : MultithreadRobotModule() {
    private val rpmTime = ElapsedTime()
    private val feederTime = ElapsedTime()
    private val encoderFailureDetectionTime = ElapsedTime()

    @Config
    internal object ShooterConfig {
        @JvmField var servoTime = 137.0
        @JvmField var servoReturnMultiplier = 2.6
        @JvmField var lowRpm = 3470.0
        @JvmField var highRpm = 4000.0
        @JvmField var timeRpm = 150.0
        @JvmField var feederClose = 0.0735
        @JvmField var feederOpen = 0.35
        @JvmField var kP = 58.0
        @JvmField var kI = 0.001 //0.03
        @JvmField var kD = 0.05
        @JvmField var kF = 14.89
        @JvmField var kF_referenceVoltage = 12.485
    }

    private lateinit var shooterMotor: DcMotorEx
    private lateinit var voltageSensor: VoltageSensor
    private lateinit var feeder: ExpansionHubServo
    private val shooterVelocitySender = CommandSender { p: Double -> shooterMotor.velocity = p }
    private val feederPositionSender = CommandSender { p: Double -> feeder.position = p }
    private var shooterMode = ShooterMode.OFF
    private var ringsToShoot: Int = 0
    private var timeToAccelerateMs = 1.0
    private var accelerationIncrement = 1.0
    var rpmTarget = 6000.0
        private set
    private var currentVelocity = 0.0
    private var velocityTarget = 2400.0
    var encoderFailureMode = false
        private set

    override fun initialize() {
        voltageSensor = WoENHardware.expansionHubVoltageSensor
        shooterMotor = WoENHardware.shooterMotor
        val motorConfigurationType = shooterMotor.motorType.clone()
        motorConfigurationType.achieveableMaxRPMFraction = 0.896476253
        motorConfigurationType.ticksPerRev = 24.0
        motorConfigurationType.gearing = 1.0
        motorConfigurationType.maxRPM = 6000.0
        shooterMotor.motorType = motorConfigurationType
        try {
            shooterMotor.setVelocityPIDFCoefficients(
                ShooterConfig.kP,
                ShooterConfig.kI,
                ShooterConfig.kD,
                ShooterConfig.kF * ShooterConfig.kF_referenceVoltage / voltageSensor.voltage
            )
        } catch (e: UnsupportedOperationException) {
            opMode.telemetry.addData("Shooter PIDF error ", e.message)
        }
        shooterMotor.direction = DcMotorSimple.Direction.FORWARD
        shooterMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        shooterMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        shootingMode = ShooterMode.OFF
        initializedservo()
        feederTime.reset()
    }

    private fun initializedservo() {
        feeder = opMode.hardwareMap.get(Servo::class.java, "feeder") as ExpansionHubServo
        feeder.position = ShooterConfig.feederClose
    }

    override fun start() {
        feeder.position = ShooterConfig.feederClose
        shooterMotor.velocity = 0.0
        shootingMode = ShooterMode.OFF
        ringsToShoot = 0
    }

    override fun updateControlHub() {
        if (ringsToShoot > 0 && feederTime.milliseconds() > ShooterConfig.servoTime * ShooterConfig.servoReturnMultiplier) {
            feedRing()
            ringsToShoot--
        }
        setFeederPosition(feederTime.milliseconds() < ShooterConfig.servoTime && velocityTarget != 0.0)
    }
    override fun updateExpansionHub() {
        currentVelocity =
            if (rpmTime.milliseconds() >= timeToAccelerateMs) velocityTarget else rpmTime.milliseconds() * accelerationIncrement * velocityTarget
        shooterVelocitySender.send(currentVelocity)
        if (encoderFailureDetectionTime.seconds() > 1) if (velocityTarget == 0.0 || currentRpm != 0.0) encoderFailureDetectionTime.reset()
        if (velocityTarget != 0.0 && ringsToShoot == 0) updatePIDFCoeffs(
            encoderFailureDetectionTime.seconds() > 3
        )
    }

    override fun updateOther() {
    }

    private val PIDFUpdateTimer = ElapsedTime()
    private fun updatePIDFCoeffs(encoderFailureMode: Boolean) {
        if (encoderFailureMode != this.encoderFailureMode || PIDFUpdateTimer.seconds() > 3.0) {
            this.encoderFailureMode = encoderFailureMode
            PIDFUpdateTimer.reset()
            try {
                if (this.encoderFailureMode) shooterMotor.setVelocityPIDFCoefficients(
                    0.0,
                    0.0,
                    0.0,
                    ShooterConfig.kF * ShooterConfig.kF_referenceVoltage / voltageSensor.voltage
                ) else shooterMotor.setVelocityPIDFCoefficients(
                    ShooterConfig.kP,
                    ShooterConfig.kI,
                    ShooterConfig.kD,
                    ShooterConfig.kF * ShooterConfig.kF_referenceVoltage / voltageSensor.voltage
                )
            } catch (ignored: UnsupportedOperationException) {
            }
        }
    }

    private fun setFeederPosition(push: Boolean) {
        feederPositionSender.send(if (push) ShooterConfig.feederOpen else ShooterConfig.feederClose)
    }

    private fun setShootersetings(Rpm: Double, time: Double) {
        if (Rpm != rpmTarget || time != timeToAccelerateMs) {
            rpmTarget = Rpm
            if (time != 0.0) timeToAccelerateMs = Math.abs(time)
            accelerationIncrement = rpmTarget / timeToAccelerateMs / 6000
            velocityTarget = rpmTarget * 0.4
        }
    }

    /*  public void onshooter(boolean On) {
        if (On)
            rpmTime.reset();
        shooterIsOn = On;
    }*/
  //  val isCorrectRpm: Boolean
       // get() = isCorrectRpm(25.0)
    val currentRpm: Double
        get() = shooterMotor.velocity * 2.5
    var shootingMode: ShooterMode
        get() = shooterMode
        set(mode) {
            if (mode != ShooterMode.OFF && shooterMode == ShooterMode.OFF) rpmTime.reset()
            shooterMode = mode
            when (mode) {
                ShooterMode.HIGHGOAL -> setShootersetings(
                    ShooterConfig.highRpm,
                    ShooterConfig.timeRpm
                )
                ShooterMode.POWERSHOT -> setShootersetings(
                    ShooterConfig.lowRpm,
                    ShooterConfig.timeRpm
                )
                ShooterMode.OFF -> setShootersetings(0.0, ShooterConfig.timeRpm)
            }
        }

    fun isCorrectRpm(error: Double = 25.0): Boolean {
        return if (encoderFailureMode) true else Math.abs(currentVelocity - shooterMotor.velocity) < error
    }

    fun feedRing() {
        //  ringsToShoot = 1
        feederTime.reset()
    }

    fun feedRings() {
        ringsToShoot = 3
    }

    enum class ShooterMode {
        HIGHGOAL, POWERSHOT, OFF
    }

}