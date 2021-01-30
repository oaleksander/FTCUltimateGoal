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

    private ExpansionHubServo close = null;
    private final CommandSender closePositionSender = new CommandSender(p -> close.setPosition(p));
    private ExpansionHubServo angle = null;
    private final CommandSender anglePositionSender = new CommandSender(p -> angle.setPosition(p));
    private LinearOpMode opMode;
    private boolean isDown = false;
    private Position posAngle = Position.UP;

    public void setOpMode(LinearOpMode OpMode) {
        opMode = OpMode;
    }

    public void initialize() {
        close = (ExpansionHubServo) opMode.hardwareMap.get(Servo.class, "wobbleGrabber");
        angle = (ExpansionHubServo) opMode.hardwareMap.get(Servo.class, "angle");
        close.setPosition(WobbleServoPositions.gripperClose);
        grabWobble(false);
        angle.setPosition(WobbleServoPositions.angleUp);
        setAngle(Position.UP);
    }

    public void grabWobble(boolean dograb) {
        closePositionSender.send(dograb ? WobbleServoPositions.gripperClose : WobbleServoPositions.gripperOpen);
    }

    public void reset() {
        close.setPosition(WobbleServoPositions.gripperClose);
        angle.setPosition(WobbleServoPositions.angleUp);
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
                setAngleServoPosition(WobbleServoPositions.angleUp);
                break;
            case DOWN:
                setAngleServoPosition(WobbleServoPositions.angleDown);
                break;
            case MEDIUM:
                setAngleServoPosition(WobbleServoPositions.angleMedium);
                break;
        }
    }

    private void setAngleServoPosition(double posa) {
        anglePositionSender.send(posa);
    }
}
