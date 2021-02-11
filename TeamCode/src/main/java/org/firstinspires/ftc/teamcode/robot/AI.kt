package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.robot.Conveyor1.conveyorm
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveRearLeft
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveRearRight
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveFrontRight
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveFrontLeft
import org.firstinspires.ftc.teamcode.robot.ThreeWheelOdometry.odometerYL
import org.firstinspires.ftc.teamcode.robot.ThreeWheelOdometry.odometerYR
import org.firstinspires.ftc.teamcode.robot.rpm.shooterMotor
import org.openftc.revextensions2.ExpansionHubMotor

@Deprecated("")
@Disabled
class AI {
    private val AItime = ElapsedTime()
    private val interval = 1000.0
    private val timeDiagnostic = 3000.0
    private val timeWait = 500.0
    private val maxError = 3.0
    private var motor0: ExpansionHubMotor? = null
    private var motor1: ExpansionHubMotor? = null
    private var motor2: ExpansionHubMotor? = null
    private var motor3: ExpansionHubMotor? = null
    private var DriveFrontLeft: ExpansionHubMotor? = null
    private var DriveFrontRight: ExpansionHubMotor? = null
    private var DriveRearLeft: ExpansionHubMotor? = null
    private var DriveRearRight: ExpansionHubMotor? = null
    fun initialize() {
        motor0 = odometerYL as ExpansionHubMotor
        motor1 = odometerYR as ExpansionHubMotor
        motor2 = conveyorm as ExpansionHubMotor
        motor3 = shooterMotor as ExpansionHubMotor
        DriveFrontLeft = driveFrontLeft as ExpansionHubMotor
        DriveFrontRight = driveFrontRight as ExpansionHubMotor
        DriveRearLeft = driveRearLeft as ExpansionHubMotor
        DriveRearRight = driveRearRight as ExpansionHubMotor
        AItime.reset()
        motor2!!.isBridgeOverTemp
    }

    fun update() {}
    fun diagnosticConveyor(): Boolean {
        AItime.reset()
        conveyorm.power = 1.0
        do {
            if (conveyorm.getCurrent(CurrentUnit.AMPS) > 0.5 && AItime.milliseconds() > timeWait) {
                conveyorm.power = 0.0
                return true
            }
        } while (WoENrobot.opMode.opModeIsActive() && AItime.milliseconds() < timeDiagnostic)
        conveyorm.power = 0.0
        return false
    }
}