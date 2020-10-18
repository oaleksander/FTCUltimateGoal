package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

import static java.lang.Math.toRadians;

@Autonomous(name = "Pos test")
public class TestPos extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.getInstance().forceInitRobot(this);
        WoENrobot.getInstance().tFdetector.initialize();
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
        WoENrobot.getInstance().tFdetectorThread.start();
        WoENrobot.getInstance().startRobot();
        WoENrobot.getInstance().tFdetector.retrieveResult();
        WoENrobot.odometry.setRobotCoordinates(new Pose2D (0,0,0));
/*------------------------------------------------------------------------------------------------*/
// AUTO CODE GOES HERE
/*------------------------------------------------------------------------------------------------*/
        WoENrobot.getInstance().drivetrain.Pos(new Pose2D(0,0, toRadians(0)));
        WoENrobot.getInstance().drivetrain.Pos(new Pose2D(0,0, toRadians(180)));
        WoENrobot.getInstance().drivetrain.Pos(new Pose2D(15,30, toRadians(180)));
        WoENrobot.getInstance().drivetrain.Pos(new Pose2D(15,30, toRadians(90)));
        while(opModeIsActive())
        {

            WoENrobot.getInstance().drivetrain.holonomicMove(0,0,0);
/*------------------------------------------------------------------------------------------------*/
// FINISH LOOP CODE GOES HERE
/*------------------------------------------------------------------------------------------------*/
            telemetry.addData("Status", "Program finished ("+getRuntime()+")");
            telemetry.update();
        }

    }
}
