package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

public class rpm implements RobotModule {

    public static DcMotorEx shooterMotor = null;
    public static ElapsedTime rpmtime = new ElapsedTime();
    public static LinearOpMode opMode;
    public static double rpm2 = 0;
    static double time_ms;
    static double x = 1;
    public static double rpm = 6000;
    static boolean on = false;
    private static boolean isActivated;
    double speedold = 0;
    double speed = 0;

    public void setOpMode(LinearOpMode OpMode) {
        opMode = OpMode;
    }

    public void initialize() {

        shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");
        MotorConfigurationType motorConfigurationType = shooterMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(0.95);
        motorConfigurationType.setTicksPerRev(24);
        motorConfigurationType.setGearing(1);
        motorConfigurationType.setMaxRPM(6000);
        shooterMotor.setMotorType(motorConfigurationType);
        PIDFCoefficients pidNew = new PIDFCoefficients(10.5, 0.9, 0.1, 0);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void reset() {
        shooterMotor.setVelocity(0);
        on=false;
    }

    public void update() {
        if (on) {
            if (time_ms > rpmtime.milliseconds()) {
                shooterpower(rpmtime.milliseconds() * x);
            }
            if (time_ms < rpmtime.milliseconds()) {
                speed = rpm * 0.4;
                if (speedold != speed) {
                    shooterVelocity(speed);
                    speedold = speed;
                }
            }
        } else {
            speed = 0;
            if (speedold != speed) {
                shooterpower(speed);
                speedold = speed;
            }
        }
    }

    public void shooterpower(double power) {
        shooterMotor.setPower(power);
    }
    public void shooterVelocity(double velocity)
    {
       // shooterMotor.setPower(velocity/2400);
        shooterMotor.setVelocity(velocity);
    }

    public void onshooter(boolean On) {
        if(On!=on) {
            rpmtime.reset();
            on = On;
        }
    }


    public void setShootersetings(double Rpm, double time) {
        rpm = Rpm;
            time_ms = time;
            x = rpm / Math.abs(time_ms) / 6000;
    }
    public boolean isCorrectRpm()
    {
        return isCorrectRpm(25);
    }
    public boolean isCorrectRpm(double error){
        if(Math.abs(speed - shooterMotor.getVelocity()) < error)
            return  true;
        else
            return false;
    }
}
