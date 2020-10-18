package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class WobbleManipulator {

    public DcMotor lever = null;
    public ElapsedTime levertime = new ElapsedTime();
    public Servo close = null;
    public void initialize(){
        lever = WoENrobot.getInstance().opMode.hardwareMap.get(DcMotor.class, "lever");
        close = WoENrobot.getInstance().opMode.hardwareMap.get(Servo.class, "close");
        lever.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void setposclose(boolean dograb){
        if (dograb) close.setPosition(0.5);
        else close.setPosition(1);
    }
    static final double minerror = 30, maxspeed = 1, kofP = 0.004, kofd = 0.007;
    double power = 0,  P = 0, D = 0, errorOld = 0,error = 0, pos = 0;
    public void update(){
        error = pos-lever.getCurrentPosition();
        if (Math.abs(error)>minerror) {
            P = error * kofP;
            D = (error - errorOld) * kofd;
            power = P + D;
            if (power > maxspeed) power = maxspeed;
            if (power < maxspeed) power = -maxspeed;
            lever.setPower(power);
            errorOld = error;
        }
        else
        {
            lever.setPower(0);
        }
    }
    public  void setposlever(double pos){
        this.pos = pos;

    }

}
