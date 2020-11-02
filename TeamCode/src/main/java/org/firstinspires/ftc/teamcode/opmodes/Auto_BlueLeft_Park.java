package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

import static java.lang.Math.toRadians;

@Autonomous(name = "Auto Blue Left Park")
public class Auto_BlueLeft_Park extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.forceInitRobot(this);
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
        WoENrobot.odometry.setRobotCoordinates(new Pose2D(-125, -156.5, toRadians(0)));
        WoENrobot.startRobot();
        WoENrobot.drivetrain.Pos(new Pose2D(-125, 25, toRadians(180)));
        while (opModeIsActive()) {
            /*------------------------------------------------------------------------------------------------*/
// FINISH LOOP CODE GOES HERE
            /*------------------------------------------------------------------------------------------------*/
            telemetry.addData("Status", "Program finished (" + getRuntime() + ")");
            telemetry.update();
        }
    }
}
