package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.misc.CommandSender;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

public class WobbleManipulator2 implements RobotModule {
    private final double closeClose = 0.73;
    private final double closeOpen = 0.19;
    private final double angleDown = 0.18;
    private final double angleMedium = 0.6;
    private final double angleUp = 1;
    private Servo close = null;
    private final CommandSender closePositionSender = new CommandSender(p -> close.setPosition(p));
    private Servo angle = null;
    private final CommandSender anglePositionSender = new CommandSender(p -> angle.setPosition(p));
    private LinearOpMode opMode;
    private boolean isGrabbed = false;
    private boolean isMed = false;
    private boolean isDown = false;
    private byte posAngle = 0;

    public void setOpMode(LinearOpMode OpMode) {
        opMode = OpMode;
    }

    public void initialize() {
        close = opMode.hardwareMap.get(Servo.class, "wobbleGrabber");
        angle = opMode.hardwareMap.get(Servo.class, "angle");
        close.setPosition(closeClose);
        isGrabbed = true;
        angle.setPosition(angleUp);
    }

    public void setposclose(boolean dograb) {
        closePositionSender.send(dograb ? closeClose : closeOpen);
    }

    public void reset() {
        isGrabbed = false;
        close.setPosition(closeClose);
        angle.setPosition(angleUp);
    }


    public void upmediumdown(boolean upmedium, boolean updown) {
        if (upmedium && !updown) {
            if (!isMed) {
                isMed = true;
                if (posAngle != 0) {
                    posAngle = 0;
                    changepos(WobbleManipulator2.positions.up);
                }
            }
        } else isMed = false;
        if (updown && !upmedium) {
            if (!isDown) {
                isDown = true;
                if (posAngle == 0 || posAngle == 2) {
                    posAngle = 1;
                    changepos(WobbleManipulator2.positions.medium);
                } else {
                    posAngle = 2;
                    changepos(WobbleManipulator2.positions.down);
                }
            }
        } else isDown = false;
    }

    public void changepos(WobbleManipulator2.positions Positions) {
        switch (Positions) {
            case up:
                setAngle(angleUp);
                break;
            case down:
                setAngle(angleDown);
                break;
            case medium:
                setAngle(angleMedium);
                break;
        }

    }

    public void setAngle(double posa) {
        anglePositionSender.send(posa);
    }

    public enum positions {up, down, medium}
}
