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

        WoENrobot.forceInitRobot(this);
      //  WoENrobot.tFdetector.initialize();
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
       // WoENrobot.tFdetectorThread.start();
        WoENrobot.odometry.setRobotCoordinates(new Pose2D(62.5, -156.5, toRadians(180)));
        WoENrobot.startRobot();
        WoENrobot.wobbleManipulator.setposclose(true);
        WoENrobot.wobbleManipulator.setposlever(0);
        WoENrobot.drivetrain.Pos(new Pose2D(93.75, -142, toRadians(180)));
        WoENrobot.delay(2000);
        /*------------------------------------------------------------------------------------------------*/
// AUTO CODE GOES HERE
        /*------------------------------------------------------------------------------------------------*/
        /*switch (WoENrobot.tFdetector.retrieveResult()) {
            case 4:
                WoENrobot.drivetrain.Pos(new Pose2D(150, 126, toRadians(180)));
                break;
            case 1:
                WoENrobot.drivetrain.Pos(new Pose2D(91, 66, toRadians(180)));
                break;
            default:
                WoENrobot.drivetrain.Pos(new Pose2D(150, 6, toRadians(180)));
        } */
        WoENrobot.wobbleManipulator.setposlever(780);
        WoENrobot.delay(1200);
        WoENrobot.wobbleManipulator.setposclose(false);
        WoENrobot.delay(500);
        WoENrobot.drivetrain.Pos(new Pose2D(62.5, 25, toRadians(180)));
        WoENrobot.wobbleManipulator.setposlever(360);
        while (opModeIsActive()) {
            /*------------------------------------------------------------------------------------------------*/
// FINISH LOOP CODE GOES HERE
            /*------------------------------------------------------------------------------------------------*/
            telemetry.addData("Status", "Program finished (" + getRuntime() + ")");
            telemetry.update();
        }
    }
}
