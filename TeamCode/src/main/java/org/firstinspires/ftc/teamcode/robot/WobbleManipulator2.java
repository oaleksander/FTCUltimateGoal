package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
public class WobbleManipulator2 {
    static final double minerror = 15, maxspeed = 0.7, kofP = 0.0015, kofd = 0.00001;
    public static DcMotorEx lever = null;
    public static Servo close = null;
    // public static Servo angle = null;
    public boolean isGrabbed = true;
    public static ElapsedTime levertime = new ElapsedTime();
    static double pos = 0;
    static double power = 0, P = 0, D = 0, errorOld = 0, error = 0;
    private static LinearOpMode opMode = null;
    double oldpower = 0, oldpos = 0;

    public void setOpMode(LinearOpMode opMode) {
      //  WobbleManipulator.opMode = opMode;
    }

    public void initialize() {

        lever = opMode.hardwareMap.get(DcMotorEx.class, "lever");
        close = opMode.hardwareMap.get(Servo.class, "wobbleGrabber");
        //   angle = opMode.hardwareMap.get(Servo.class, "angle");

        lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lever.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lever.setDirection(DcMotorSimple.Direction.REVERSE);
        lever.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setposclose(true);
    }

    public void setposclose(boolean dograb) {
        if (dograb != isGrabbed) {
            isGrabbed = dograb;
            if (dograb) close.setPosition(0.85);
            else close.setPosition(0.42);
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
            /*WoENrobot.getInstance().opMode.telemetry.addData("err",error);
            WoENrobot.getInstance().opMode.telemetry.addData("pow",power);
            WoENrobot.getInstance().opMode.telemetry.addData("pos",pos);
            WoENrobot.getInstance().opMode.telemetry.addData("enc",lever.getCurrentPosition());
            WoENrobot.getInstance().opMode.telemetry.update(); */
            errorOld = error;
        } else {
            power = 0;
            if (oldpower != power) {
                lever.setPower(0);
                oldpower = power;
            }
        }
    }
    //double medium = 0.5, down = 0, up = 1, oldposangle = up;
    short posangle = 0;
    boolean ismed = false, isdown = false;
    public void upmediumdown(boolean upmedium, boolean updown){
        if (upmedium && !updown){
            if(!ismed){
                ismed = true;
                if (posangle == 1){
                    posangle = 0;
                    changepos(WobbleManipulator.positions.up);
                }
                else {
                    posangle = 1;
                    changepos(WobbleManipulator.positions.medium);
                }
            }
        }
        else ismed = false;
        if (updown && !upmedium){
            if (!isdown){
                isdown = true;
                if(posangle == 2){
                    posangle = 0;
                    changepos(WobbleManipulator.positions.up);
                }
                else{
                    posangle = 2;
                    changepos(WobbleManipulator.positions.down);
                }
            }
        }
        else isdown = false;
        /*
        if(upmedium != ismed){
            ismed = upmedium;
            isdown = false; // test
            if (upmedium) changepos(positions.up);
            else changepos(positions.medium);
        }
        if (updown != isdown){
            isdown = updown;
            ismed = false; //test
            if(updown) changepos(positions.up);
            else changepos(positions.down);
        }*/
    }
    /*
    if(gamepad2.b)
        {
            if(pushback== false)
            {
                pushback = true;
                if (speed == 1) {
                    speed = 0.5;
                } else {
                    speed = 1;
                }
            }
        }
        else
            pushback = false;
     */
    enum positions {up, down, medium}
    public void changepos(WobbleManipulator.positions Positions){
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

   /* public void setAngle(double posa){
        if (posa != oldpos){
            angle.setPosition(posa);
            oldpos = posa;
        }
    }

    */

    public void setposlever(double pos) {
        WobbleManipulator.pos = pos;
    }
}
