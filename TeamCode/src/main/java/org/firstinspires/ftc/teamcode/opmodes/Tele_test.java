package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;

@TeleOp(name = "Teleop TEST/PRACTICE", group = "Debugging")

public class Tele_test extends LinearOpMode {

    boolean buttonAwasPressed = false;

    @Override
    public void runOpMode() {
        forceInitRobot(this);

        startRobot();
        //odometry.setRobotCoordinates(new Pose2D(0, 0, 0));
        odometry.setRobotCoordinates(new Pose2D(93.75 * 1 + 31.25 * (-1), -156.5, 0));
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
                wobbleManipulator.setposlever(0);
            else if (gamepad1.y)
                wobbleManipulator.setposlever(490);
            else if (gamepad1.b)
                wobbleManipulator.setposlever(700);
            if (gamepad1.a) {
                if (!buttonAwasPressed)
                    wobbleManipulator.setposclose(!wobbleManipulator.isGrabbed);
                buttonAwasPressed = true;
            } else
                buttonAwasPressed = false;
            drivetrain.setRobotVelocity(y, x, turn);
            spinOnce();

        }
    }
}
