package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

import static java.lang.Math.toRadians;

@Autonomous(name = "Auto Red Left")
public class Auto_RedLeft extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.getInstance().forceInitRobot(this);
        WoENrobot.getInstance().tFdetector.initialize();
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
        WoENrobot.getInstance().tFdetectorThread.start();
        WoENrobot.getInstance().startRobot();
        WoENrobot.odometry.setRobotCoordinates(new Pose2D(62.5, -156.5, toRadians(180)));
        /*------------------------------------------------------------------------------------------------*/
// AUTO CODE GOES HERE
        /*------------------------------------------------------------------------------------------------*/
        switch (WoENrobot.getInstance().tFdetector.retrieveResult()) {
            case 4:
                WoENrobot.getInstance().drivetrain.Pos(new Pose2D(150, 126, toRadians(180)));
                break;
            case 1:
                WoENrobot.getInstance().drivetrain.Pos(new Pose2D(91, 66, toRadians(180)));
                break;
            default:
                WoENrobot.getInstance().drivetrain.Pos(new Pose2D(150, 6, toRadians(180)));
        }
        WoENrobot.getInstance().drivetrain.Pos(new Pose2D(62.5, 25, toRadians(180)));
        WoENrobot.getInstance().drivetrain.Pos(new Pose2D(62.5, 25, toRadians(0)));
        while (opModeIsActive()) {
            /*------------------------------------------------------------------------------------------------*/
// FINISH LOOP CODE GOES HERE
            /*------------------------------------------------------------------------------------------------*/
            telemetry.addData("Status", "Program finished (" + getRuntime() + ")");
            telemetry.update();
        }
    }
}
