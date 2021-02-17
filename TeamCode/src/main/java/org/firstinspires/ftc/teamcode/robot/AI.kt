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

class AI : RobotModule() {
    private val AItime = ElapsedTime()
    private val headTime = ElapsedTime()
    private val maxTimeHead: Double = 60000.0
    private val interval = 1000.0
    private val timeDiagnostic = 1000.0
    private val timeWait = 500.0
    private lateinit var OdometryYL: ExpansionHubMotor
    private lateinit var OdometryYR: ExpansionHubMotor
    private lateinit var Conveyorm: ExpansionHubMotor
    private lateinit var ShooterMotor: ExpansionHubMotor
    private lateinit var DriveFrontLeft: ExpansionHubMotor
    private lateinit var DriveFrontRight: ExpansionHubMotor
    private lateinit var DriveRearLeft: ExpansionHubMotor
    private lateinit var DriveRearRight: ExpansionHubMotor
    private var owerHead = false
    override fun initialize() {
        OdometryYL = WoENHardware.odometerYL
        OdometryYR = WoENHardware.odometerYR
        Conveyorm = WoENHardware.conveyorMotor
        ShooterMotor = WoENHardware.shooterMotor
        DriveFrontLeft = WoENHardware.driveFrontLeft
        DriveFrontRight = WoENHardware.driveFrontRight
        DriveRearLeft = WoENHardware.driveRearLeft
        DriveRearRight = WoENHardware.driveRearRight
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