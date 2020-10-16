package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class rpm {
public DcMotor rpm = null;
public ElapsedTime rpmtime = null;
public void runOpMode(){ }
public void intrpm(){
    rpm = WoENrobot.getInstance().opMode.hardwareMap.get(DcMotor.class, "rpm");
}
    public void startrpm(double time) {
rpmtime.reset();
double x=0;
x=6000/time;
        do {
            rpm.setPower(rpmtime.milliseconds()*x/6000);
        } while (rpmtime.milliseconds()<time && WoENrobot.getInstance().opMode.opModeIsActive());
        rpm.setPower(1);
    }
}
