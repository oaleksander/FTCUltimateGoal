package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.drivetrain;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.forceInitRobot;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.spinOnce;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator;

@TeleOp(name = "Teleop TEST/PRACTICE", group = "Debugging")

public class Tele_test extends LinearOpMode {

    boolean buttonAwasPressed = false;

    public static double SIDE_LENGTH = 20;

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    @Override
    public void runOpMode() {
        forceInitRobot(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();
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
                    wobbleManipulator.setposclose(!WobbleManipulator.isGrabbed);
                buttonAwasPressed = true;
            } else
                buttonAwasPressed = false;
            drivetrain.holonomicMove(y, x, turn);
            double by = -odometry.getRobotCoordinates().x/2.54;
            double bx = odometry.getRobotCoordinates().y/2.54;
            double l = SIDE_LENGTH / 2;

            double[] bxPoints = { l, -l, -l, l };
            double[] byPoints = { l, l, -l, -l };
            rotatePoints(bxPoints, byPoints, -odometry.getRobotCoordinates().heading);
            for (int i = 0; i < 4; i++) {
                bxPoints[i] += bx;
                byPoints[i] += by;
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setFill("black")
                    .fillPolygon(bxPoints, byPoints);
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Status", "Running");
            telemetry.addLine("encoder")
                    /*            .addData("FL", drivetrain.driveFrontLeft.getCurrentPosition())
                                 .addData("FR", drivetrain.driveFrontRight.getCurrentPosition())
                                 .addData("RL", drivetrain.driveRearLeft.getCurrentPosition())
                                 .addData("RR", drivetrain.driveRearRight.getCurrentPosition())

                      */.addData("odYL", odometry.bulkData.getMotorCurrentPosition(0))
                    .addData("odYR", odometry.bulkData.getMotorCurrentPosition(1))
                    .addData("odX", odometry.bulkData.getMotorCurrentPosition(2));
            telemetry.addLine("Control")
                    .addData("y", y)
                    .addData("x", x)
                    .addData("turn", turn);
            try {
                telemetry.addLine("odometry")
                        .addData("y", odometry.getRobotCoordinates().y)
                        .addData("x", odometry.getRobotCoordinates().x)
                        .addData("head", Math.toDegrees(odometry.getRobotCoordinates().heading));
            } catch (NullPointerException ignored) {
            }
            //telemetry.addData("Y",  WoENrobot.odometry.getRobotCoordinates().y);
            //telemetry.addData("X",  WoENrobot.odometry.getRobotCoordinates().x);
            //telemetry.addData("Heading", Math.toDegrees( WoENrobot.odometry.getRobotCoordinates().heading));
            telemetry.update(); spinOnce();

        }
    }
}
