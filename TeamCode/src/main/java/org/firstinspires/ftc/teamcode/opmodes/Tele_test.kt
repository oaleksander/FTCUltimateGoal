package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.misc.ButtonSwitch
import org.firstinspires.ftc.teamcode.misc.SinglePressButton
import org.firstinspires.ftc.teamcode.robot.WoENrobot.forceInitRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry
import org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter
import org.firstinspires.ftc.teamcode.robot.WoENrobot.spinOnce
import org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator
import org.firstinspires.ftc.teamcode.robot.rpm

@TeleOp(name = "Teleop TEST/PRACTICE", group = "Debugging")
class Tele_test : LinearOpMode() {
    var buttonAwasPressed = false
    override fun runOpMode() {
        forceInitRobot(this)
        startRobot()
        val buttonASwitch = ButtonSwitch()
        val buttonStartSwitch = ButtonSwitch()
        val threeRingPresser = SinglePressButton()
        odometry.robotCoordinates = Pose2D(0.0, 0.0, 0.0)
        buttonASwitch.isTriggered(true)
        buttonASwitch.isTriggered(false)
        while (opModeIsActive()) {
            wobbleManipulator.grabWobble(buttonASwitch.isTriggered(gamepad1.a))
            wobbleManipulator.upmediumdown(gamepad1.y, gamepad1.x) // correct
            shooter.shootingMode =
                if (buttonStartSwitch.isTriggered(gamepad1.start)) rpm.ShooterMode.HIGHGOAL else rpm.ShooterMode.OFF
            if (threeRingPresser.isTriggered(gamepad1.b)) shooter.feedRings()
            var turn = 0.0
            var y: Double
            var x: Double
            turn -= if (gamepad1.left_bumper) 0.25 else gamepad1.left_trigger.toDouble()
            turn += if (gamepad1.right_bumper) 0.25 else gamepad1.right_trigger.toDouble()
            y = -gamepad1.left_stick_y.toDouble()
            x = gamepad1.left_stick_x.toDouble()
            if (gamepad1.dpad_up) y += 1.0
            if (gamepad1.dpad_down) y = -1.0
            if (gamepad1.dpad_left) x = -1.0
            if (gamepad1.dpad_right) x += 1.0
            movement.humanSetVelocity(x, y, turn)
            telemetry.addData("y", y)
            spinOnce()
        }
    }
}