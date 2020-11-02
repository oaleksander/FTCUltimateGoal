package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

public class rpm implements RobotModule {

    public DcMotorEx shooterMotor = null;
    public ElapsedTime rpmtime = new ElapsedTime();
    private LinearOpMode opMode;

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initialize()
    {
        shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "odometerX");
    }
    static double time_ms;
    static double x = 1;
    private static boolean isActivated;

    static boolean on = false;
    public void update() {
        //rpmtime.reset();
        //double x = 1;
        if (on) {
            if (time_ms > rpmtime.milliseconds()) {
                shooterpower(rpmtime.milliseconds() * x);
            }
            if (time_ms < rpmtime.milliseconds()) {
                shooterpower(1);
            }
        }
        else {
           shooterpower(0);
        }
    }
    public void shooterpower(double power){
        shooterMotor.setPower(power);
    }
    public void onshooter(boolean on){
        this.on = on;
    }
    public void setspeedlevel(double time){
        rpm.time_ms = time;
        rpmtime.reset();
        if (time_ms > 0)
            x = 1 / time_ms;
    }
}
