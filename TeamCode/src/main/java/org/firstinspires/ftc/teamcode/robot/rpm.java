package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

public class rpm implements RobotModule {

    public DcMotor shooterMotor = null;
    public ElapsedTime rpmtime = new ElapsedTime();
    private LinearOpMode opMode;

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initialize() {
        shooterMotor = opMode.hardwareMap.get(DcMotor.class, "rpm");
    }
    static double time_ms;
    double x = 1;
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
