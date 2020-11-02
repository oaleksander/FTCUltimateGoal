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

    public void activate()
    {

    }
    public void update() {
        //rpmtime.reset();
        //double x = 1;
        if (time_ms < rpmtime.milliseconds()) {
            shooterMotor.setPower(rpmtime.milliseconds() * x);
        }
        if (time_ms > rpmtime.milliseconds()){
            shooterMotor.setPower(1);
        }
    }
    public void setspeedlevel(double time){
        rpm.time_ms = time;
        rpmtime.reset();
        if (time_ms > 0)
            x = 1 / time_ms;
    }
}
