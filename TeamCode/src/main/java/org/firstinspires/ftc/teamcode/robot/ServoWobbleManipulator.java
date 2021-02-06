package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.misc.CommandSender;
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator;
import org.openftc.revextensions2.ExpansionHubServo;

public class ServoWobbleManipulator implements WobbleManipulator {

    @Config
    static class WobbleServoPositions {
        public static double gripperClose = 0.92;
        public static double gripperOpen = 0.19;
        public static double angleDown = 0.18;
        public static double angleMedium = 0.6;
        public static double angleUp = 1;
    }

    private ExpansionHubServo gripper = null;
    private final CommandSender closePositionSender = new CommandSender(p -> gripper.setPosition(p));
    private ExpansionHubServo leverArm = null;
    private final CommandSender anglePositionSender = new CommandSender(p -> leverArm.setPosition(p));
    private LinearOpMode opMode;
    private boolean isDown = false;
    private Position posAngle = Position.UP;
    private double leverArmPosition = 0;
    private double gripperPosition = 0;

    public void setOpMode(LinearOpMode OpMode) {
        opMode = OpMode;
    }

    public void initialize() {
        gripper = (ExpansionHubServo) opMode.hardwareMap.get(Servo.class, "wobbleGrabber");
        leverArm = (ExpansionHubServo) opMode.hardwareMap.get(Servo.class, "angle");
        grabWobble(true);
        setAngle(Position.UP);
        update();
    }

    public void grabWobble(boolean dograb) {
        gripperPosition = dograb ? WobbleServoPositions.gripperClose : WobbleServoPositions.gripperOpen;
    }

    public void start() {
        update();
    }

    public void update() {
        closePositionSender.send(gripperPosition);
        anglePositionSender.send(leverArmPosition);
    }


    public void upmediumdown(boolean upmedium, boolean updown) {
        if (upmedium && !updown) {
            setAngle(Position.UP);
        } else if (updown && !upmedium) {
            if (!isDown) {
                isDown = true;
                if (posAngle != Position.MEDIUM)
                    setAngle(Position.MEDIUM);
                else
                    setAngle(Position.DOWN);
            }
        } else isDown = false;
    }

    public void setAngle(Position position) {
        posAngle = position;
        switch (position) {
            case UP:
                leverArmPosition = WobbleServoPositions.angleUp;
                break;
            case DOWN:
                leverArmPosition = WobbleServoPositions.angleDown;
                break;
            case MEDIUM:
                leverArmPosition = WobbleServoPositions.angleMedium;
                break;
        }
    }
}
