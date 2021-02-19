package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.misc.HSVRGB;
import org.firstinspires.ftc.teamcode.misc.SinglePressButton;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.FullInitWithCV;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.runTime;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.setLedColors;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot;

public class AutoOpMode extends LinearOpMode {

    MovementMacros M;
    byte xSign = 1;
    byte sideSign = 1;
    boolean thereAreTwoGamepads;
    double delayAtStart = 0;

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

    SinglePressButton delayAtStartIncrement = new SinglePressButton();
    SinglePressButton delayAtStartDecrement = new SinglePressButton();

    void start_loop() {
        Vector3D color = HSVRGB.convert((float) (runTime.seconds() * 50) % 360, 100, 50);
        setLedColors((int) color.x, (int) color.y, (int) color.z);
        String indicator = runTime.seconds()%1>0.5?(runTime.seconds()%1>0.75?"/":"|"):(runTime.seconds()%1>0.25?"—":"\\");
        telemetry.addLine("Use gamedad 1 X/B to select alliance color, dpad L/R to select alliance side, dpad UP/DOWN to change starting delay. "+indicator);
        xSign = gamepad1.b ? 1 : gamepad1.x ? -1 : xSign;
        sideSign = gamepad1.dpad_right || gamepad1.left_stick_x > 0.5 ? 1 : gamepad1.dpad_left || gamepad1.left_stick_x < -0.5 ? -1 : sideSign;
        delayAtStart = Range.clip(delayAtStart+(delayAtStartIncrement.isTriggered(gamepad1.dpad_up)?500:0)-(delayAtStartDecrement.isTriggered(gamepad1.dpad_down)?500:0),0,30000);
        telemetry.addData("Alliance", getXSign() == 1 ? "RED" : "BLUE");
        telemetry.addData("Tape Side", getSideSign() == 1 ? "RIGHT" : "LEFT");
        telemetry.addData("Starting delay [ms]",delayAtStart);
        telemetry.addLine("");
        thereAreTwoGamepads = gamepad2.start || gamepad2.b || thereAreTwoGamepads;
        telemetry.addData("OpenCV Stack size",openCVNode.getStackSize());
        if (thereAreTwoGamepads) telemetry.addLine("Second gamepad detected");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        FullInitWithCV(this);
        startRobot();
        movement.setActiveBraking(true);
        if (isStopRequested()) return;
        openCVNode.stopCam();
        MovementMacros.setSettings(getXSign(), getSideSign());
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
