package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.misc.HSVRGB;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.FullInitWithCV;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.runTime;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.setLedColors;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot;

public class AutonomousOpMode extends LinearOpMode {

    MovementMacros M;
    byte xSign = 1;
    byte sideSign = 1;
    boolean thereAreTwoGamepads;

    public void main() {
    }

    protected byte getXSign() {
        return xSign;
    }

    protected byte getSideSign() {
        return sideSign;
    }

    public Pose2D getStartPosition() {
        return new Pose2D(93.91741046 * getXSign() + 30.1416 * getSideSign(), -156.12089687, 0);

    }

    @Override
    public void waitForStart() {
        while (!isStarted()) {
            start_loop();
        }
        super.waitForStart();
    }

    void start_loop() {
        Vector3D color = HSVRGB.convert((float) (runTime.seconds() * 50) % 360, 100, 50);
        setLedColors((int) color.x, (int) color.y, (int) color.z);
        telemetry.addLine("Use gamedad 1 X/B to select alliance color, dpad L/R to select alliance side");
        xSign = gamepad1.b ? 1 : gamepad1.x ? -1 : xSign;
        sideSign = gamepad1.dpad_right || gamepad1.left_stick_x > 0.5 ? 1 : gamepad1.dpad_left || gamepad1.left_stick_x < -0.5 ? -1 : sideSign;
        telemetry.addData("Alliance", getXSign() == 1 ? "RED" : "BLUE");
        telemetry.addData("SIDE", getSideSign() == 1 ? "RIGHT" : "LEFT");
        thereAreTwoGamepads = gamepad2.start || gamepad2.b || thereAreTwoGamepads;
        if (thereAreTwoGamepads) telemetry.addLine("Second gamepad detected");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        FullInitWithCV(this);
        startRobot();
        if(isStopRequested()) return;
        openCVNode.stopCam();
        new MovementMacros(getXSign(), getSideSign());
        odometry.setRobotCoordinates(getStartPosition());
        try {
            main();
        } finally {
            setLedColors(0, 128, 128);
            telemetry.addData("Status", "Program finished (" + getRuntime() + ")");
            telemetry.update();
        }
    }

}
