package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.robot.Conveyor1.conveyorm
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveFrontLeft
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveFrontRight
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveRearRight
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveRearLeft
import org.firstinspires.ftc.teamcode.robot.ThreeWheelOdometry.odometerYL
import org.firstinspires.ftc.teamcode.robot.ThreeWheelOdometry.odometerYR
import org.firstinspires.ftc.teamcode.robot.rpm.shooterMotor
import org.firstinspires.ftc.teamcode.superclasses.RobotModule
import org.openftc.revextensions2.ExpansionHubMotor

class AI : RobotModule {
    private var opMode: LinearOpMode? = null
    private val AItime = ElapsedTime()
    private val headTime = ElapsedTime()
    private val maxTimeHead: Double = 60000.0
    private val interval = 1000.0
    private val timeDiagnostic = 1000.0
    private val timeWait = 500.0
    private var OdometryYL: ExpansionHubMotor? = null
    private var OdometryYR: ExpansionHubMotor? = null
    private var Conveyorm: ExpansionHubMotor? = null
    private var ShooterMotor: ExpansionHubMotor? = null
    private var DriveFrontLeft: ExpansionHubMotor? = null
    private var DriveFrontRight: ExpansionHubMotor? = null
    private var DriveRearLeft: ExpansionHubMotor? = null
    private var DriveRearRight: ExpansionHubMotor? = null
    private var owerHead = false
    override fun setOpMode(OpMode: LinearOpMode) {
        opMode = OpMode
    }
    override fun initialize() {
        OdometryYL = odometerYL as ExpansionHubMotor
        OdometryYR = odometerYR as ExpansionHubMotor
        Conveyorm = conveyorm as ExpansionHubMotor
        ShooterMotor = shooterMotor as ExpansionHubMotor
        DriveFrontLeft = driveFrontLeft as ExpansionHubMotor
        DriveFrontRight = driveFrontRight as ExpansionHubMotor
        DriveRearLeft = driveRearLeft as ExpansionHubMotor
        DriveRearRight = driveRearRight as ExpansionHubMotor
        AItime.reset()
    }

    /*override fun update() {
        if (AItime.milliseconds() > timeDiagnostic) {
            AItime.reset()
            if((Conveyorm?.let { tempMotor(it) }!! || ShooterMotor?.let { tempMotor(it) }!! || OdometryYL?.let { tempMotor(it) }!! || OdometryYR?.let { tempMotor(it) }!! ||
                    DriveFrontLeft?.let { tempMotor(it) }!! || DriveFrontRight?.let { tempMotor(it) }!! || DriveRearLeft?.let { tempMotor(it) }!! || DriveRearRight?.let { tempMotor(it) }!!) && owerHead) {
                owerHead = true
                headTime.reset()
                telemetry.addData("Warning!", "owerhead")
            }
            else if(headTime.milliseconds() > maxTimeHead){
                    //break
                }
                else {
                owerHead = false
            }
        }
    }*/
    private fun tempMotor(motor: ExpansionHubMotor): Boolean {
        return motor.isBridgeOverTemp
    }
    fun diagnositcServo(servo: Servo, startPos :Double, endPos :Double) {
        AItime.reset()
        servo.position = startPos
        while (opMode!!.opModeIsActive() && AItime.milliseconds() < timeDiagnostic/2) {}
        servo.position = endPos
        while (opMode!!.opModeIsActive() && AItime.milliseconds() < timeDiagnostic) {}
    }
    fun diagnosticRange(sensor: DistanceSensor ) : Boolean {
        return !sensor.getDistance(DistanceUnit.CM).isNaN()
    }

    fun diagnosticMotor(motor: DcMotorEx): Boolean {
        AItime.reset()
        motor.power = 1.0
        do {
            if (motor.getCurrent(CurrentUnit.AMPS) > 0.5 && AItime.milliseconds() > timeWait) {
                motor.power = 0.0
                return true
            }
        } while (opMode!!.opModeIsActive() && AItime.milliseconds() < timeDiagnostic)
        motor.power = 0.0
        return false
    }

}