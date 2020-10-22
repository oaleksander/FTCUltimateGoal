package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

import static java.lang.Math.toRadians;

@Autonomous(name = "Auto Blue Right Park")
public class Auto_BlueRight_Park extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.getInstance().forceInitRobot(this);
        WoENrobot.getInstance().tFdetector.initialize();
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
        WoENrobot.getInstance().tFdetectorThread.start();
        WoENrobot.odometry.setRobotCoordinates(new Pose2D(-62.5, -156.5, toRadians(180)));
        WoENrobot.getInstance().startRobot();
        WoENrobot.getInstance().drivetrain.Pos(new Pose2D(-62.5, 25, toRadians(180)));
        while (opModeIsActive()) {
            /*------------------------------------------------------------------------------------------------*/
// FINISH LOOP CODE GOES HERE
            /*------------------------------------------------------------------------------------------------*/
            telemetry.addData("Status", "Program finished (" + getRuntime() + ")");
            telemetry.update();
        }
    }
}
