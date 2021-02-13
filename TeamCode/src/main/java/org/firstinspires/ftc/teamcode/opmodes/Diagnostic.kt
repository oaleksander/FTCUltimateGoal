package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.WoENrobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.forceInitRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.ai

class Diagnostic : LinearOpMode() {
    override fun runOpMode() {
        forceInitRobot(this)
        startRobot()
        telemetry.addData("shooter", ai.diagnosticShooter())
        telemetry.addData("Conveyor", ai.diagnosticConveyor())
        telemetry.update()
    }
}