package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.WoENrobot;

@TeleOp(name="Teleop TEST/PRACTICE", group="Debugging")
public class Tele_test extends LinearOpMode {
    @Override
    public void runOpMode() {

        //initRobot();
        WoENrobot.getInstance().forceInitRobot(this);
        WoENrobot.getInstance().startRobot();
        //odometry.start();
        while(opModeIsActive())
        {
            double turn = 0;
            if(gamepad1.left_bumper) turn -=0.25;
            else turn -= gamepad1.left_trigger;
            if(gamepad1.right_bumper) turn +=0.25;
            else turn += gamepad1.right_trigger;
            WoENrobot.drivetrain.holonomicMove(-gamepad1.left_stick_y, gamepad1.left_stick_x, turn);
            double f = gamepad1.right_stick_y;
            telemetry.addData("Status", "Running");
            telemetry.addData("Y",  WoENrobot.odometry.getRobotCoordinates().y);
            telemetry.addData("X",  WoENrobot.odometry.getRobotCoordinates().x);
            telemetry.addData("Heading", Math.toDegrees( WoENrobot.odometry.getRobotCoordinates().heading));
            telemetry.update();

        }
    }
}
