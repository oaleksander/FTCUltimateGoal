package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

public class WobbleManipulator2 implements RobotModule {
    public static Servo close = null;
    public static Servo angle = null;
    public boolean isGrabbed = false;
    public static ElapsedTime levertime = new ElapsedTime();
    private static LinearOpMode opMode = null;
    double oldpos = 0;

    public void setOpMode(LinearOpMode opMode) {
        WobbleManipulator2.opMode = opMode;
    }

    public void initialize() {
        close = opMode.hardwareMap.get(Servo.class, "wobbleGrabber");
        angle = opMode.hardwareMap.get(Servo.class, "angle");
        close.setPosition(0.85);
        isGrabbed = true;
        angle.setPosition(1);
        oldpos = 1;
    }

    public void setposclose(boolean dograb) {
        if (dograb != isGrabbed) {
            isGrabbed = dograb;
            if (dograb) close.setPosition(0.73);
            else close.setPosition(0.19);
        }
    }

    public void reset()
    {
        isGrabbed = false;
        close.setPosition(0.73);
        angle.setPosition(1);
    }

    short posangle = 0;
    boolean ismed = false, isdown = false;
    public void upmediumdown(boolean upmedium, boolean updown){
        if (upmedium && !updown){
            if(!ismed){
                ismed = true;
                if (posangle != 0){
                    posangle = 0;
                    changepos(WobbleManipulator2.positions.up);
                }
            }
        }
        else ismed = false;
        if (updown && !upmedium){
            if (!isdown){
                isdown = true;
                if(posangle == 0 || posangle == 2){
                    posangle = 1;
                    changepos(WobbleManipulator2.positions.medium);
                }
                else{
                    posangle = 2;
                    changepos(WobbleManipulator2.positions.down);
                }
            }
        }
        else isdown = false;
    }
    public enum positions {up, down, medium}
    public void changepos(WobbleManipulator2.positions Positions){
        switch (Positions){
            case up:
                setAngle(1);
                break;
            case down:
                setAngle(0.18);
                break;
            case medium:
                setAngle(0.6);
                break;
        }

    }

    public void setAngle(double posa){
        if (posa != oldpos){
            angle.setPosition(posa);
            oldpos = posa;
        }
    }
}
