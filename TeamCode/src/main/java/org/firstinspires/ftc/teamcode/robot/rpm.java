package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

public class rpm implements RobotModule {

    public static DcMotorEx shooterMotor = null;
    public static ElapsedTime rpmtime = new ElapsedTime();
    public static LinearOpMode opMode;

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initialize()
    {
        shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "odometerX");
        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetshooter();
    }
    static double time_ms;
    static double x = 1;
    private static boolean isActivated;
    static double rpm = 6000;
    static boolean on = false;
    public static double rpm2 = 0;
    double speedold = 0,speed = 0;
    public static double power2 = 0; //delete
    public void update() {
        posold = pos;
        pos = shooterMotor.getCurrentPosition();
        oldtime = time;
        time = rpmtime.milliseconds();
        posold = pos - posold;
        oldtime = oldtime - time;
         rpm2 = posold / oldtime * 2500;// что выводит?
        power2 = regulator();// что выводит?
        if (on) {
            if (time_ms > rpmtime.milliseconds()) {
                shooterpower(rpmtime.milliseconds() * x);
            }
            if (time_ms < rpmtime.milliseconds()) {
                speed = rpm / 6000;
                if (speedold!=speed) {
                    shooterpower(speed);
                    speedold = speed;
                }
            }
        }
        else {
            speed = 0;
            if (speedold != speed){
                shooterpower(speed);
                speedold = speed;
            }
        }
    }
    public void shooterpower(double power){
        shooterMotor.setPower(power);
     //   shooterMotor.setVelocity(power*1000);
    }
    public void onshooter(boolean on){
        if(on == !this.on) {
            this.on = on;      //Времено: что бы можно было вкл/выкл в цикле с геймпада
            rpmtime.reset();
        }
       // rpmtime.reset();
    }
    double pos = 0, posold = 0,time = 0, oldtime = 0, error = 0, P = 0, D = 0, errorold = 0,power = 0, maxpower = 0;
    static final double kofP = 0.0015, kofd = 0.00001;
    public double regulator(){
        error = rpm - rpm2;
        P = error * kofP;
        D = (error - errorold)* kofd;
        errorold = error;
        power = D + P;
        if (power > maxpower) power = maxpower;
        if (power < -maxpower) power = -maxpower;
        return  power;

    }
    public void resetshooter(){
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setrpm(double rpm){
        this.rpm = rpm;
        maxpower = rpm/6000;
    }
    public void setspeedlevel(double time){
        this.time_ms = time;
        if (time_ms > 0)
            x = rpm / time_ms / 6000;
    }
}
