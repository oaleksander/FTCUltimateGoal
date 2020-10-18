package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

import static java.lang.Math.toRadians;

@Autonomous(name = "Experimental Auto Red Right")
public class Auto_ExperimentalRedLeft extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.getInstance().forceInitRobot(this);
        WoENrobot.getInstance().tFdetector.initialize();
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
        WoENrobot.getInstance().tFdetectorThread.start();
        WoENrobot.odometry.setRobotCoordinates(new Pose2D(62.5, -156.5, toRadians(180)));
        WoENrobot.getInstance().startRobot();
        WoENrobot.getInstance().wobbleManipulator.setposclose(true);
        WoENrobot.getInstance().wobbleManipulator.setposlever(0);
        WoENrobot.getInstance().drivetrain.Pos(new Pose2D(93.75, -142, toRadians(180)));
        WoENrobot.getInstance().delay(2000);
        /*------------------------------------------------------------------------------------------------*/
// AUTO CODE GOES HERE
        /*------------------------------------------------------------------------------------------------*/
        switch (WoENrobot.getInstance().tFdetector.retrieveResult()) {
            case 4:
                WoENrobot.getInstance().drivetrain.Pos(new Pose2D(150, 156, toRadians(180)));
                break;
            case 1:
                WoENrobot.getInstance().drivetrain.Pos(new Pose2D(91, 96, toRadians(180)));
                break;
            default:
                WoENrobot.getInstance().drivetrain.Pos(new Pose2D(150, 36, toRadians(180)));
        }

        WoENrobot.getInstance().wobbleManipulator.setposclose(false);
        WoENrobot.getInstance().delay(500);
        WoENrobot.getInstance().drivetrain.Pos(new Pose2D(93.75, 145, toRadians(180)));
        WoENrobot.getInstance().wobbleManipulator.setposlever(580);
        WoENrobot.getInstance().delay(1000);
        WoENrobot.getInstance().drivetrain.Pos(new Pose2D(62.5, 25, toRadians(180)));
        while (opModeIsActive()) {
            /*------------------------------------------------------------------------------------------------*/
// FINISH LOOP CODE GOES HERE
            /*------------------------------------------------------------------------------------------------*/
            telemetry.addData("Status", "Program finished (" + getRuntime() + ")");
            telemetry.update();
        }
    }
}
