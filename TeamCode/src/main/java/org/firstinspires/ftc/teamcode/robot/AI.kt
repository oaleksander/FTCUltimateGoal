package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
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

@Deprecated("")
class AI : RobotModule {
    private var opMode: LinearOpMode? = null
    private val AItime = ElapsedTime()
    private val interval = 1000.0
    private val timeDiagnostic = 3000.0
    private val timeWait = 500.0
    private var OdometerYL: ExpansionHubMotor? = null
    private var OdometryYR: ExpansionHubMotor? = null
    private var Conveyorm: ExpansionHubMotor? = null
    private var ShooterMotor: ExpansionHubMotor? = null
    private var DriveFrontLeft: ExpansionHubMotor? = null
    private var DriveFrontRight: ExpansionHubMotor? = null
    private var DriveRearLeft: ExpansionHubMotor? = null
    private var DriveRearRight: ExpansionHubMotor? = null
    override fun setOpMode(OpMode: LinearOpMode) {
        opMode = OpMode
    }
    override fun initialize() {
        OdometerYL = odometerYL as ExpansionHubMotor
        OdometryYR = odometerYR as ExpansionHubMotor
        Conveyorm = conveyorm as ExpansionHubMotor
        ShooterMotor = shooterMotor as ExpansionHubMotor
        DriveFrontLeft = driveFrontLeft as ExpansionHubMotor
        DriveFrontRight = driveFrontRight as ExpansionHubMotor
        DriveRearLeft = driveRearLeft as ExpansionHubMotor
        DriveRearRight = driveRearRight as ExpansionHubMotor
        AItime.reset()
    }

   /* override fun update() {
        if (AItime.milliseconds() > timeDiagnostic) {
            tempConveyor()
            tempShooter()
            AItime.reset()
        }
    }
*/
    fun tempConveyor(): Boolean {
        return Conveyorm!!.isBridgeOverTemp
    }

    fun tempMotor(motor: DcMotorEx): Boolean {
        val motorExp = motor as ExpansionHubMotor
        return motorExp.isBridgeOverTemp
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