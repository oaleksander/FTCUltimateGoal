package org.firstinspires.ftc.teamcode.robot;

import android.icu.lang.UCharacter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

public class WobbleManipulator implements RobotModule{

    public static DcMotorEx lever = null;
    public ElapsedTime levertime = new ElapsedTime();
    public static Servo close = null;

    private LinearOpMode opMode = null;
    public void setOpMode(LinearOpMode opMode) {this.opMode = opMode;}

    public void initialize(){
        lever = opMode.hardwareMap.get(DcMotorEx.class, "lever");
        close = opMode.hardwareMap.get(Servo.class, "wobbleGrabber");

        lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lever.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lever.setDirection(DcMotorSimple.Direction.REVERSE);
        lever.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public static boolean isGrabbed = false;
    public void setposclose(boolean dograb){
        isGrabbed = dograb;
        if (dograb) close.setPosition(0.5);
        else close.setPosition(1);
    }
    static final double minerror = 15, maxspeed = 1, kofP = 0.0015, kofd = 0.00001;
    double power = 0,  P = 0, D = 0, errorOld = 0,error = 0;
    static double pos = 0;
    public void update(){
        error = pos-lever.getCurrentPosition();
        if (Math.abs(error)>minerror) {
            P = error * kofP;
            D = (error - errorOld) * kofd;
            power = P + D;
            if (power > maxspeed) power = maxspeed;
            if (power < -maxspeed) power = -maxspeed;
            lever.setPower(power);
            /*WoENrobot.getInstance().opMode.telemetry.addData("err",error);
            WoENrobot.getInstance().opMode.telemetry.addData("pow",power);
            WoENrobot.getInstance().opMode.telemetry.addData("pos",pos);
            WoENrobot.getInstance().opMode.telemetry.addData("enc",lever.getCurrentPosition());
            WoENrobot.getInstance().opMode.telemetry.update(); */
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
