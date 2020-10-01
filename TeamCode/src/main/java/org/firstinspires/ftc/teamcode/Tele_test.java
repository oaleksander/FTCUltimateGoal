package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop TEST/PRACTICE", group="")
public class Tele_test extends WoENrobot{
    @Override
    public void runOpMode() {

        //initRobot();
        forceInitRobot();
        startRobot();
        //odometry.start();
        autoManipulator.start();
        while(opModeIsActive())
        {
            double turn = 0;
            if(gamepad1.left_bumper) turn -=0.25;
            else turn -= gamepad1.left_trigger;
            if(gamepad1.right_bumper) turn +=0.25;
            else turn += gamepad1.right_trigger;
            omniMoveYX(-gamepad1.left_stick_y, gamepad1.left_stick_x, turn);
            if(gamepad1.a) doGrabStone = true;
            if(gamepad1.b) doGrabStone = false;
            double f = gamepad1.right_stick_y;
            liftMotorPowers(f,f);
            telemetry.addData("Status", "Running");
            telemetry.addData("Y", returnYCoordinate());
            telemetry.addData("1", limitSwitchFront.getState());
            telemetry.addData("2", limitSwitchRear.getState());
            telemetry.addData("X", returnXCoordinate());
            telemetry.addData("Heading", returnOrientation());
            telemetry.update();

        }
    }
}
