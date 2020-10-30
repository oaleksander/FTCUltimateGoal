package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

import static java.lang.Math.toRadians;

@Autonomous(name = "Ring Auto Blue Right")
public class Auto_ExperimentalBlueRight extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.forceInitRobot(this);
        //WoENrobot.tFdetector.initialize();
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
     //   WoENrobot.tFdetectorThread.start();
        WoENrobot.odometry.setRobotCoordinates(new Pose2D(-62.5, -156.5, toRadians(180)));
        WoENrobot.startRobot();
        WoENrobot.wobbleManipulator.setposclose(true);
        WoENrobot.wobbleManipulator.setposlever(0);
        WoENrobot.drivetrain.Pos(new Pose2D(-87, -135, toRadians(180)));
        WoENrobot.delay(1600);
        /*------------------------------------------------------------------------------------------------*/
// AUTO CODE GOES HERE
        /*------------------------------------------------------------------------------------------------*/
       /* switch (WoENrobot.tFdetector.retrieveResult()) {
            case 4:
                WoENrobot.drivetrain.Pos(new Pose2D(-150, 156, toRadians(-90)));
                break;
            case 1:
                WoENrobot.drivetrain.Pos(new Pose2D(-91, 96, toRadians(180)));
                break;
            default:
                WoENrobot.drivetrain.Pos(new Pose2D(-150, 36, toRadians(180)));
        } */

        WoENrobot.wobbleManipulator.setposclose(false);
        WoENrobot.wobbleManipulator.setposlever(120);
        WoENrobot.delay(100);
        WoENrobot.wobbleManipulator.setposlever(0);
        WoENrobot.delay(100);
        WoENrobot.wobbleManipulator.setposlever(120);
        WoENrobot.drivetrain.Pos(new Pose2D(-82.5, 150, toRadians(180)));
        WoENrobot.wobbleManipulator.setposlever(505);
        WoENrobot.delay(1000);
        WoENrobot.wobbleManipulator.setposlever(470);
        WoENrobot.drivetrain.Pos(new Pose2D(-62.5, 25, toRadians(180)));
        while (opModeIsActive()) {
            /*------------------------------------------------------------------------------------------------*/
// FINISH LOOP CODE GOES HERE
            /*------------------------------------------------------------------------------------------------*/
            telemetry.addData("Status", "Program finished (" + getRuntime() + ")");
            telemetry.update();
        }
    }
}
