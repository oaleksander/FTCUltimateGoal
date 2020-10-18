package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

import static java.lang.Math.toRadians;

@TeleOp(name="Teleop TEST/PRACTICE", group="Debugging")
public class Tele_test extends LinearOpMode {

    boolean buttonAwasPressed = false;
    @Override
    public void runOpMode() {

        WoENrobot.getInstance().forceInitRobot(this);
        WoENrobot.getInstance().startRobot();
        WoENrobot.odometry.setRobotCoordinates(new Pose2D(62.5, -156.5, toRadians(180)));
        while(opModeIsActive())
        {
            double turn = 0;
            double y = 0;
            double x = 0;
            if(gamepad1.left_bumper) turn -=0.25;
            else turn -= gamepad1.left_trigger;
            if(gamepad1.right_bumper) turn +=0.25;
            else turn += gamepad1.right_trigger;
            y=-gamepad1.left_stick_y;
            x= gamepad1.left_stick_x;
            if (gamepad1.dpad_up)
                y+=1;
            if (gamepad1.dpad_down)
                y=-1;
            if (gamepad1.dpad_left)
                x=-1;
            if (gamepad1.dpad_right)
                x+=1;
            if(gamepad1.x)
                WoENrobot.wobbleManipulator.setposlever(0);
            else if(gamepad1.y)
                WoENrobot.wobbleManipulator.setposlever(490);
            else if(gamepad1.b)
                WoENrobot.wobbleManipulator.setposlever(720);
            if(gamepad1.a) {
                if(!buttonAwasPressed)
                WoENrobot.wobbleManipulator.setposclose(!WoENrobot.wobbleManipulator.isGrabbed);
                buttonAwasPressed = true;
            }
            else
                buttonAwasPressed = false;
            WoENrobot.drivetrain.holonomicMove(-y, -x, turn);
            telemetry.addData("Status", "Running");
            telemetry.addLine("encoder")
                    .addData("FL", Drivetrain.driveFrontLeft.getCurrentPosition())
                    .addData("FR", Drivetrain.driveFrontRight.getCurrentPosition())
                    .addData("RL", Drivetrain.driveRearLeft.getCurrentPosition())
                    .addData("RR", Drivetrain.driveRearRight.getCurrentPosition())
                    .addData("odYL", WoENrobot.odometry.bulkData.getMotorCurrentPosition(0))
                    .addData("odYR", -WoENrobot.odometry.bulkData.getMotorCurrentPosition(1))
                    .addData("odX", WoENrobot.odometry.bulkData.getMotorCurrentPosition(2));
            telemetry.addLine("Control")
                    .addData("y", y)
                    .addData("x", x)
                    .addData("turn", turn);
            try{
                telemetry.addLine("odometry")
                        .addData("y", WoENrobot.odometry.getRobotCoordinates().y)
                        .addData("x", WoENrobot.odometry.getRobotCoordinates().x)
                        .addData("head", Math.toDegrees(WoENrobot.odometry.getRobotCoordinates().heading));
            }
            catch(NullPointerException ignored) {}
            //telemetry.addData("Y",  WoENrobot.odometry.getRobotCoordinates().y);
            //telemetry.addData("X",  WoENrobot.odometry.getRobotCoordinates().x);
            //telemetry.addData("Heading", Math.toDegrees( WoENrobot.odometry.getRobotCoordinates().heading));
            telemetry.update();

        }
    }
}
