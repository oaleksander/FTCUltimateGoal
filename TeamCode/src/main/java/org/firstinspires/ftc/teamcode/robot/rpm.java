package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

public class rpm implements RobotModule {
    private LinearOpMode opMode;

    private DcMotorEx shooterMotor = null;

    private final ElapsedTime rpmTime = new ElapsedTime();

    private boolean on = false;

    private double time_ms = 1;
    private double x = 1;
    private double rpm = 6000;
    private double speedOld = 0;
    private double speed = 0;

    public void setOpMode(LinearOpMode OpMode) {
        opMode = OpMode;
    }

    public void initialize() {

        shooterMotor = opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");
        MotorConfigurationType motorConfigurationType = shooterMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(0.896476253);
        motorConfigurationType.setTicksPerRev(24);
        motorConfigurationType.setGearing(1);
        motorConfigurationType.setMaxRPM(6000);
        shooterMotor.setMotorType(motorConfigurationType);
       // PIDFCoefficients pidNew = new PIDFCoefficients(1, 1, 9, 0);
//       PIDFCoefficients pidNew = new PIDFCoefficients(3.628, 1.3, 5, 14.28);
         PIDFCoefficients pidNew = new PIDFCoefficients(25.5, 0.075, 16, 15.23);
        shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidNew);
        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void reset() {
        shooterMotor.setVelocity(0);
        on=false;
    }

    public void update() {
        if (on) {
            if (time_ms > rpmTime.milliseconds()) {
                shooterpower(rpmTime.milliseconds() * x);
            }
            if (time_ms < rpmTime.milliseconds()) {
                speed = rpm * 0.4;
                if (speedOld != speed) {
                    shooterVelocity(speed);
                    speedOld = speed;
                }
            }
        } else {
            speed = 0;
            if (speedOld != speed) {
                shooterpower(speed);
                speedOld = speed;
            }
        }
    }

    private void shooterpower(double power) {
        shooterMotor.setPower(power);
    }

    private void shooterVelocity(double velocity)
    {
        shooterMotor.setVelocity(velocity);
    }

    public void onshooter(boolean On) {
        if(On!=on) {
            rpmTime.reset();
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

    public double getRpm(){
        return shooterMotor.getVelocity()*2.5;
    }

    public boolean isCorrectRpm(double error){
        return Math.abs(speed - shooterMotor.getVelocity()) < error;
    }
}
