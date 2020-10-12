package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class rpm extends LinearOpMode {
public DcMotor rpm = null;
public ElapsedTime rpmtime = null;
public void runOpMode(){ }
public void intrpm(){
    rpm = hardwareMap.get(DcMotor.class, "rpm");
}
    public void startrpm(double time) {
rpmtime.reset();
double x=0,timem=0,timex=0,cof=0,power=0;
x=time/6000;
        do {
           timex=rpmtime.milliseconds()-timem;
            timem=rpmtime.milliseconds();
            cof=timex*x;
            power+=cof;
            rpm.setPower(power/6000);
        } while (rpmtime.milliseconds()<time && opModeIsActive());
        rpm.setPower(1);
    }
}
