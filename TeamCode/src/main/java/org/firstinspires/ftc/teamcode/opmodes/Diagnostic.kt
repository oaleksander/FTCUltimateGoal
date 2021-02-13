package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.Conveyor1.conveyorm
import org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.forceInitRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.ai
import org.firstinspires.ftc.teamcode.robot.rpm.shooterMotor
import org.firstinspires.ftc.teamcode.robot.ThreeWheelOdometry.odometerYL
import org.firstinspires.ftc.teamcode.robot.ThreeWheelOdometry.odometerYR
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveFrontLeft
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveFrontRight
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveRearRight
import org.firstinspires.ftc.teamcode.robot.MecanumDrivetrain.driveRearLeft

class Diagnostic : LinearOpMode() {
    override fun runOpMode() {
        forceInitRobot(this)
        startRobot()
        telemetry.addData("shooter", ai.diagnosticMotor(shooterMotor))
        telemetry.addData("Conveyor", ai.diagnosticMotor(conveyorm))
        telemetry.addData("OdometerYL", ai.diagnosticMotor(odometerYL))
        telemetry.addData("odometerYR", ai.diagnosticMotor(odometerYR))
        telemetry.addData("driveRearLeft", ai.diagnosticMotor(driveRearLeft))
        telemetry.addData("driveRearRight", ai.diagnosticMotor(driveRearRight))
        telemetry.addData("driveFrontRight", ai.diagnosticMotor(driveFrontRight))
        telemetry.addData("driveFrontLeft", ai.diagnosticMotor(driveFrontLeft))
        telemetry.update()
    }
}