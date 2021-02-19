package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.misc.ButtonSwitch
import org.firstinspires.ftc.teamcode.misc.SinglePressButton
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootHighGoalAsync
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootPOWERSHOTAngle
import org.firstinspires.ftc.teamcode.robot.WoENrobot.*
import org.firstinspires.ftc.teamcode.robot.rpm

@TeleOp(name = "TeleOp COMPETITION", group = "Competition")
class Tele_competition : LinearOpMode() {
    override fun runOpMode() {
        initRobot(this)
        startRobot()
        val buttonASwitch = ButtonSwitch()
        val buttonBackSwitch = ButtonSwitch()
        val shooterSpeedSwitch = ButtonSwitch()
        val shooterOnOffSwitch = ButtonSwitch()
        val threeRingPresser = SinglePressButton()
        while (opModeIsActive()) {
            if (gamepad2.left_bumper) {
                ShootHighGoalAsync()
            }
            if (!movement.pathFollowerIsActive()) {
                wobbleManipulator.grabWobble(buttonASwitch.isTriggered(gamepad2.right_bumper))
                wobbleManipulator.upmediumdown(gamepad2.b, gamepad2.x) // correct
                conveyor.setConveyorPower(if (buttonBackSwitch.isTriggered(gamepad2.left_trigger > 0.5)) 1.0 else 0.0)
                shooter.shootingMode = if (shooterOnOffSwitch.isTriggered(gamepad2.a)) if (shooterSpeedSwitch.isTriggered(gamepad2.y)) rpm.ShooterMode.POWERSHOT else rpm.ShooterMode.HIGHGOAL else rpm.ShooterMode.OFF
                conveyor.setForceReverse(gamepad2.right_trigger > 0.5)
                if (gamepad1.x) ShootPOWERSHOTAngle()
                if (gamepad1.a) shooter.feedRing()
                if (threeRingPresser.isTriggered(gamepad1.right_stick_button)) shooter.feedRings()
            }
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
            spinOnce()
        }
    }
}