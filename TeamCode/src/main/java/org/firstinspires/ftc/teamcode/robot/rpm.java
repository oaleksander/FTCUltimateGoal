package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

public class rpm implements RobotModule {

    public static DcMotorEx shooterMotor = null;
    public static ElapsedTime rpmtime = new ElapsedTime();
    public static LinearOpMode opMode;
    public static double rpm2 = 0;
    static double time_ms;
    static double x = 1;
    static double rpm = 6000;
    static boolean on = false;
    private static boolean isActivated;
    double speedold = 0, speed = 0;
    double pos = 0, posold = 0, time = 0, oldtime = 0;

    public void setOpMode(LinearOpMode opMode) {
        org.firstinspires.ftc.teamcode.robot.rpm.opMode = opMode;
    }

    public void initialize() {
        shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "lever");
        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetshooter();
    }

    public void update() {
       /* posold = pos;
        pos = shooterMotor.getCurrentPosition();
        oldtime = time;
        time = rpmtime.milliseconds();
        posold = pos - posold;
        oldtime = oldtime - time;
        rpm2 = posold / oldtime * 2500;// что выводит?
        if (on) {
            if (time_ms > rpmtime.milliseconds()) {
                shooterpower(rpmtime.milliseconds() * x);
            }
            if (time_ms < rpmtime.milliseconds()) {
                speed = rpm / 6000;
                if (speedold != speed) {
                    shooterpower(speed);
                    speedold = speed;
                }
            }
        } else {
            speed = 0;
            if (speedold != speed) {
                shooterpower(speed);
                speedold = speed;
            }
        }*/
    }

    public void shooterpower(double power) {
        //shooterMotor.setPower(power);
        shooterMotor.setVelocity(power*0.4);
    }

    public void onshooter(boolean on) {
        if (on == !org.firstinspires.ftc.teamcode.robot.rpm.on) {
            org.firstinspires.ftc.teamcode.robot.rpm.on = on;      //Времено: что бы можно было вкл/выкл в цикле с геймпада
            rpmtime.reset();
        }
        // rpmtime.reset();
    }

    public void resetshooter() {
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setrpm(double rpm) {
        org.firstinspires.ftc.teamcode.robot.rpm.rpm = rpm;
    }

    public void setspeedlevel(double time) {
        time_ms = time;
        if (time_ms > 0)
            x = rpm / time_ms / 6000;
    }
}
