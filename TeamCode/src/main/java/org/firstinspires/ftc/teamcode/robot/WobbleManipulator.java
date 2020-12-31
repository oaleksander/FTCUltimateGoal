
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
@Deprecated
public class WobbleManipulator implements RobotModule {
    private LinearOpMode opMode;

    private DcMotorEx lever = null;

    private Servo close = null;

    private ElapsedTime leverTime = new ElapsedTime();

    private boolean ismed = false, isdown = false;
    private boolean isGrabbed = true;

    private byte posangle = 0;

    private double pos = 0;
    private double power = 0, P = 0, D = 0, errorOld = 0, error = 0;
    private double oldpower = 0;

    private final double closeClose = 0.73;
    private final double closeOpen = 0.19;
    private final double minerror = 15, maxspeed = 0.7, kofP = 0.0015, kofd = 0.00001;

    public void setOpMode(LinearOpMode OpMode) {
        opMode = OpMode;
    }

    public void initialize() {

        lever = opMode.hardwareMap.get(DcMotorEx.class, "lever");
        close = opMode.hardwareMap.get(Servo.class, "wobbleGrabber");


        lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lever.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lever.setDirection(DcMotorSimple.Direction.REVERSE);
        lever.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setposclose(true);
    }

    public void setposclose(boolean dograb) {
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

    public void upmediumdown(boolean upmedium, boolean updown){
        if (upmedium && !updown){
            if(!ismed){
                ismed = true;
                if (posangle == 1){
                    posangle = 0;
                    changepos(positions.up);
                }
                else {
                    posangle = 1;
                    changepos(positions.medium);
                }
            }
        }
        else ismed = false;
        if (updown && !upmedium){
            if (!isdown){
                isdown = true;
                if(posangle == 2){
                    posangle = 0;
                    changepos(positions.up);
                }
                else{
                    posangle = 2;
                    changepos(positions.down);
                }
            }
        }
        else isdown = false;
    }

    enum positions {up, down, medium}
    public void changepos(positions Positions){
        switch (Positions){
            case up:
                setposlever(0);
                break;
            case down:
                setposlever(920);
                break;
            case medium:
                setposlever(550);
                break;
        }

    }

    public void setposlever(double Pos) {
        pos = Pos;
    }

}
