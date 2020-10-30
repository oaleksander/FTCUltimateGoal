package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator;

@TeleOp(name = "TeleOp COMPETITION", group = "Competition")
public class Tele_competition extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.initRobot(this);
        WoENrobot.startRobot();
        boolean buttonAwasPressed = false;
        while (opModeIsActive()) {
            double turn = 0;
            double y = 0;
            double x = 0;
            if (gamepad1.left_bumper) turn -= 0.25;
            else turn -= gamepad1.left_trigger;
            if (gamepad1.right_bumper) turn += 0.25;
            else turn += gamepad1.right_trigger;
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            if (gamepad1.dpad_up)
                y += 1;
            if (gamepad1.dpad_down)
                y = -1;
            if (gamepad1.dpad_left)
                x = -1;
            if (gamepad1.dpad_right)
                x += 1;
            if (gamepad1.x)
                WoENrobot.wobbleManipulator.setposlever(0);
            else if (gamepad1.y)
                WoENrobot.wobbleManipulator.setposlever(490);
            else if (gamepad1.b)
                WoENrobot.wobbleManipulator.setposlever(700);
            if (gamepad1.a) {
                if (!buttonAwasPressed)
                    WoENrobot.wobbleManipulator.setposclose(!WobbleManipulator.isGrabbed);
                buttonAwasPressed = true;
            } else
                buttonAwasPressed = false;
            WoENrobot.drivetrain.holonomicMove(-y, -x, turn);
        }
    }
}
