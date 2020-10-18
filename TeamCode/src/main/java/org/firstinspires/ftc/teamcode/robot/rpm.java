package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class rpm {

public DcMotor shooterMotor = null;
public ElapsedTime rpmtime = new ElapsedTime();
public void initialize(){
    shooterMotor = WoENrobot.getInstance().opMode.hardwareMap.get(DcMotor.class, "rpm");
}
    public void startrpm(double time_ms) {
rpmtime.reset();
double x=1;
if(time_ms>0)
x=1/time_ms;
        do {
            shooterMotor.setPower(rpmtime.milliseconds()*x);
        } while (rpmtime.milliseconds()<time_ms && WoENrobot.getInstance().opMode.opModeIsActive());
        shooterMotor.setPower(1);
    }
}
