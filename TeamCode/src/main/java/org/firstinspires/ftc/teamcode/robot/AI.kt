package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import org.openftc.revextensions2.ExpansionHubMotor

class AI : MultithreadRobotModule() {
    private val AItime = ElapsedTime()
    private val AItimeControlHub = ElapsedTime()
    private val AItimeExpansionHub = ElapsedTime()
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
        AItimeExpansionHub.reset()
        AItimeControlHub.reset()
    }

  /*  override fun updateControlHub() {
       if (AItimeControlHub.milliseconds() > timeDiagnostic) {
           AItimeControlHub.reset()
           if (tempMotor(DriveFrontLeft) || tempMotor(DriveFrontRight) || tempMotor(DriveRearLeft) || tempMotor(DriveRearRight)) {
               telemetry.addData("Warning!", "owerHeadControlHub")
           }
       }
    }

    override fun updateExpansionHub() {
        if (AItimeExpansionHub.milliseconds() > timeDiagnostic){
            AItimeExpansionHub.reset()
            if (tempMotor(Conveyorm) || tempMotor(ShooterMotor) || tempMotor(OdometryYL) || tempMotor(OdometryYR)){
                telemetry.addData("Warning!", "owerHeadExpansionHub")
            }
        }
    }
*/
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