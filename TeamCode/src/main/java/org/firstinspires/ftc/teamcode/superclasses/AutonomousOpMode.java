package org.firstinspires.ftc.teamcode.superclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;
public class AutonomousOpMode extends LinearOpMode {

    public void main() {
    }

    protected byte getXSign() {
        return 1;
    }

    protected byte getSideSign() {
        return 1;
    }

    Pose2D getStartPosition() {
        return new Pose2D(93.75 * getXSign() + 31.25 * getSideSign(), -156.5, 0);

    }

    @Override
    public void runOpMode() {
        new WoENrobot();
        FullInitWithCV(this);
        odometry.setRobotCoordinates(getStartPosition());
        main();
        setLedColors(0,128,128);
        telemetry.addData("Status", "Program finished (" + getRuntime() + ")");
        telemetry.update();
    }

}
