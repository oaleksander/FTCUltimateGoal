package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator;

@Deprecated
public class MotorWobbleManipulator extends RobotModule implements WobbleManipulator {
    private final double closeClose = 0.73;
    private final double closeOpen = 0.19;
    private final double minerror = 15, maxspeed = 0.7, kofP = 0.0015, kofd = 0.00001;
    private final ElapsedTime leverTime = new ElapsedTime();
    private DcMotorEx lever = null;
    private Servo close = null;
    private boolean ismed = false, isdown = false;
    private boolean isGrabbed = true;
    private byte posangle = 0;
    private double pos = 0;
    private double power = 0, P = 0, D = 0, errorOld = 0, error = 0;
    private double oldpower = 0;

    public void initialize() {

        lever = WoENHardware.INSTANCE.getLever();
        close = WoENHardware.INSTANCE.getGripper();


        lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lever.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lever.setDirection(DcMotorSimple.Direction.REVERSE);
        lever.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabWobble(true);
    }

    public void grabWobble(boolean dograb) {
        if (dograb != isGrabbed) {
            isGrabbed = dograb;
            if (dograb) close.setPosition(closeClose);
            else close.setPosition(closeOpen);
        }
    }

    public void update() {
        error = pos - lever.getCurrentPosition();
        if (Math.abs(error) > minerror) {
            P = error * kofP;
            D = (error - errorOld) * kofd;
            power = P + D;
            if (power > maxspeed) power = maxspeed;
            if (power < -maxspeed) power = -maxspeed;
            if (oldpower != power) {
                lever.setPower(power);
                oldpower = power;
            }
            errorOld = error;
        } else {
            power = 0;
            if (oldpower != power) {
                lever.setPower(0);
                oldpower = power;
            }
        }
    }

    public void upmediumdown(boolean upmedium, boolean updown) {
        if (upmedium && !updown) {
            if (!ismed) {
                ismed = true;
                if (posangle == 1) {
                    posangle = 0;
                    setAngle(Position.UP);
                } else {
                    posangle = 1;
                    setAngle(Position.MEDIUM);
                }
            }
        } else ismed = false;
        if (updown && !upmedium) {
            if (!isdown) {
                isdown = true;
                if (posangle == 2) {
                    posangle = 0;
                    setAngle(Position.UP);
                } else {
                    posangle = 2;
                    setAngle(Position.DOWN);
                }
            }
        } else isdown = false;
    }

    public void setAngle(Position Positions) {
        switch (Positions) {
            case UP:
                setposlever(0);
                break;
            case DOWN:
                setposlever(920);
                break;
            case MEDIUM:
                setposlever(550);
                break;
        }

    }

    public void setposlever(double Pos) {
        pos = Pos;
    }

}
