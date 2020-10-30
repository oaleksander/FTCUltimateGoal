package org.firstinspires.ftc.teamcode.superclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

public class AutonomousOpMode extends LinearOpMode {

    public void mainProgram() {
    }

    int getXSign() {
        return 1;
    }

    int getSideSign() {
        return 1;
    }

    Pose2D getStartPosition() {
        return new Pose2D(93.75 * getXSign() + 31.25 * getSideSign(), -156.5, 0);

    }

    @Override
    public void runOpMode() {
        new WoENrobot();
        WoENrobot.FullInitWithCV(this);
        WoENrobot.odometry.setRobotCoordinates(getStartPosition());
        mainProgram();
        telemetry.addData("Status", "Program finished (" + getRuntime() + ")");
        telemetry.update();
    }

}
