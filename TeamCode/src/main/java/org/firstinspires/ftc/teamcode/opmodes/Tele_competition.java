package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.ButtonSwitch;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;

@TeleOp(name = "TeleOp COMPETITION", group = "Competition")
public class Tele_competition extends LinearOpMode {
    @Override
    public void runOpMode() {
        initRobot(this);
        startRobot();
        ButtonSwitch buttonAswitch = new ButtonSwitch();

        ButtonSwitch buttonStartswitch = new ButtonSwitch();

        //odometry.setRobotCoordinates(new Pose2D(0, 0, 0));
        odometry.setRobotCoordinates(new Pose2D(93.75 * 1 + 31.25 * (-1), -156.5, 0));
        shooter.setShootersetings(3333,1000);
        buttonAswitch.isTriggered(true);
        buttonAswitch.isTriggered(false);
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
            wobbleManipulator.setposclose(buttonAswitch.isTriggered(gamepad1.a));
            wobbleManipulator.upmediumdown(gamepad1.y,gamepad1.x); // correct
            shooter.onshooter(buttonStartswitch.isTriggered(gamepad1.start));
            if(gamepad1.b)
                conveyor.feedRing();

            drivetrain.setRobotVelocity(y, x, turn);
      //      spinOnce();
        }
    }
}
