package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.WoENrobot;

@TeleOp(name="Teleop TEST/PRACTICE", group="")
public class Tele_test extends WoENrobot {
    @Override
    public void runOpMode() {

        //initRobot();
        forceInitRobot();
        startRobot();
        //odometry.start();
        while(opModeIsActive())
        {
            double turn = 0;
            if(gamepad1.left_bumper) turn -=0.25;
            else turn -= gamepad1.left_trigger;
            if(gamepad1.right_bumper) turn +=0.25;
            else turn += gamepad1.right_trigger;
            omniMoveYX(-gamepad1.left_stick_y, gamepad1.left_stick_x, turn);
            double f = gamepad1.right_stick_y;
            telemetry.addData("Status", "Running");
            telemetry.addData("Y", odometry.getRobotCoordinates().y);
            telemetry.addData("X", odometry.getRobotCoordinates().x);
            telemetry.addData("Heading", Math.toDegrees(odometry.getRobotCoordinates().heading));
            telemetry.update();

        }
    }
}
