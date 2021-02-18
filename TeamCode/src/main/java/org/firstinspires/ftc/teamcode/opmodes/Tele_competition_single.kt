package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.misc.ButtonSwitch
import org.firstinspires.ftc.teamcode.misc.SinglePressButton
import org.firstinspires.ftc.teamcode.robot.WoENrobot
import org.firstinspires.ftc.teamcode.robot.rpm

@TeleOp(name = "TeleOp COMPETITION single", group = "Competition")
class Tele_competition_single : LinearOpMode() {
    override fun runOpMode() {
        WoENrobot.initRobot(this)
        WoENrobot.startRobot()
        val buttonAswitch = ButtonSwitch()
        val buttonBackswitch = ButtonSwitch()
        val buttonStartswitch = ButtonSwitch()
        val threeRingPresser = SinglePressButton()

        //shooter.setShootersetings(3800, 500);
        while (opModeIsActive()) {
            WoENrobot.wobbleManipulator.grabWobble(buttonAswitch.isTriggered(gamepad1.a))
            WoENrobot.wobbleManipulator.upmediumdown(gamepad1.y, gamepad1.x) // correct
            WoENrobot.shooter.shootingMode =
                if (buttonStartswitch.isTriggered(gamepad1.start)) rpm.ShooterMode.HIGHGOAL else rpm.ShooterMode.OFF
            WoENrobot.conveyor.setConveyorPower(if (buttonBackswitch.isTriggered(gamepad1.back)) 1.0 else 0.0)
            if (gamepad1.b) WoENrobot.shooter.feedRing()
            if (threeRingPresser.isTriggered(gamepad1.right_stick_button)) WoENrobot.shooter.feedRings()
            var turn = 0.0
            var y: Double
            var x: Double
            turn -= if (gamepad1.left_bumper) 0.25 else gamepad1.left_trigger.toDouble()
            turn += if (gamepad1.right_bumper) 0.25 else gamepad1.right_trigger.toDouble()
            //  turn +=gamepad1.right_stick_x;
            y = -gamepad1.left_stick_y.toDouble()
            x = gamepad1.left_stick_x.toDouble()
            if (gamepad1.dpad_up) y += 1.0
            if (gamepad1.dpad_down) y = -1.0
            if (gamepad1.dpad_left) x = -1.0
            if (gamepad1.dpad_right) x += 1.0
            WoENrobot.movement.humanSetVelocity(x, y, turn)
            WoENrobot.spinOnce()
        }
    }
}