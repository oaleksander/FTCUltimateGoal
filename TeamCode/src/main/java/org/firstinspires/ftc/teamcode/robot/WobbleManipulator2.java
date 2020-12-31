package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

public class WobbleManipulator2 implements RobotModule {
    private Servo close = null;
    private Servo angle = null;

    private LinearOpMode opMode;

    private boolean isGrabbed = false;
    private boolean isMed = false;
    private boolean isDown = false;

    private byte posAngle = 0;

    private double oldPos = 0;

    private final double closeClose = 0.73;
    private final double closeOpen = 0.19;
    private final double angleDown = 0.18;
    private final double angleMedium = 0.6;
    private final double angleUp = 1;

    public void setOpMode(LinearOpMode OpMode) {
        opMode = OpMode;
    }

    public void initialize() {
        close = opMode.hardwareMap.get(Servo.class, "wobbleGrabber");
        angle = opMode.hardwareMap.get(Servo.class, "angle");
        close.setPosition(closeClose);
        isGrabbed = true;
        angle.setPosition(angleUp);
        oldPos = 1;
    }

    public void setposclose(boolean dograb) {
        if (dograb != isGrabbed) {
            isGrabbed = dograb;
            if (dograb) close.setPosition(closeClose);
            else close.setPosition(closeOpen);
        }
    }

    public void reset()
    {
        isGrabbed = false;
        close.setPosition(closeClose);
        angle.setPosition(angleUp);
    }


    public void upmediumdown(boolean upmedium, boolean updown){
        if (upmedium && !updown){
            if(!isMed){
                isMed = true;
                if (posAngle != 0){
                    posAngle = 0;
                    changepos(WobbleManipulator2.positions.up);
                }
            }
        }
        else isMed = false;
        if (updown && !upmedium){
            if (!isDown){
                isDown = true;
                if(posAngle == 0 || posAngle == 2){
                    posAngle = 1;
                    changepos(WobbleManipulator2.positions.medium);
                }
                else{
                    posAngle = 2;
                    changepos(WobbleManipulator2.positions.down);
                }
            }
        }
        else isDown = false;
    }
    public enum positions {up, down, medium}
    public void changepos(WobbleManipulator2.positions Positions){
        switch (Positions){
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

    public void setAngle(double posa){
        if (posa != oldPos){
            angle.setPosition(posa);
            oldPos = posa;
        }
    }
}
