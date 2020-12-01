package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.signum;

public class MecanumDrivetrain implements RobotModule, Drivetrain {


    private boolean smartMode = true;
    private static final double maxRampPerSec = 1 / 0.25;
    /* Drivetrain constatnts. */
    public static double maxDriveSpeed = 1;
    public static double minDriveSpeed = 0.05;
    /* Drivetrain hardware members. */
    // public static ExpansionHubEx expansionHub1 = null;
    public static DcMotorEx driveFrontLeft = null;
    public static DcMotorEx driveFrontRight = null;
    public static DcMotorEx driveRearLeft = null;
    public static DcMotorEx driveRearRight = null;
    double powerFrontLeft_requested = 0;
    double powerFrontRight_requested = 0;
    double powerRearLeft_requested = 0;
    double powerRearRight_requested = 0;
    double powerFrontLeft_current = powerFrontLeft_requested;
    double powerFrontRight_current = powerFrontRight_requested;
    double powerRearLeft_current = powerRearLeft_requested;
    double powerRearRight_current = powerRearRight_requested;
    double powerFrontLeft_old = powerFrontLeft_requested;
    double powerFrontRight_old = powerFrontRight_requested;
    double powerRearLeft_old = powerRearRight_requested;
    double powerRearRight_old = powerRearRight_requested;
    private final ElapsedTime looptime = new ElapsedTime();

    public MecanumDrivetrain(){
    }

    private LinearOpMode opMode = null;

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initialize() {
        assignNames();
        setMotorDirections();
        setMotor0PowerBehaviors(DcMotorEx.ZeroPowerBehavior.FLOAT);
        setSmartMode(true);
        setRobotVelocity(0,0,0);
    }

    public void setSmartMode(boolean SmartMode)
    {
        smartMode = SmartMode;
        if(smartMode)
        {
            setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        else
        {
            setMotorMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void assignNames() {
        driveFrontLeft = opMode.hardwareMap.get(DcMotorEx.class, "driveFrontLeft");
        driveFrontRight = opMode.hardwareMap.get(DcMotorEx.class, "driveFrontRight");
        driveRearLeft = opMode.hardwareMap.get(DcMotorEx.class, "driveRearLeft");
        driveRearRight = opMode.hardwareMap.get(DcMotorEx.class, "driveRearRight");
    }

    private void setMotorDirections() {
        driveFrontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        driveFrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        driveRearLeft.setDirection(DcMotorEx.Direction.FORWARD);
        driveRearRight.setDirection(DcMotorEx.Direction.REVERSE);
    }

    private void setMotor0PowerBehaviors(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        driveFrontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        driveFrontRight.setZeroPowerBehavior(zeroPowerBehavior);
        driveRearLeft.setZeroPowerBehavior(zeroPowerBehavior);
        driveRearRight.setZeroPowerBehavior(zeroPowerBehavior);
    }

    private void setMotorMode(DcMotorEx.RunMode runMode) {
        driveFrontLeft.setMode(runMode);
        driveFrontRight.setMode(runMode);
        driveRearLeft.setMode(runMode);
        driveRearRight.setMode(runMode);
    }

    private void resetEncoders() {
        setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMaxDriveSpeed(double value) {
        maxDriveSpeed = clip(abs(value), 0, 1);
    }

    public void setMinDriveSpeed(double value) {
        minDriveSpeed = clip(abs(value), 0, 1);
    }

    public void update() {
        if (powerFrontLeft_requested == 0) {
            powerFrontLeft_current = powerFrontLeft_old = 0;
            //driveFrontLeft.setPower(powerFrontLeft_current);

        } else {
            powerFrontLeft_current += min(abs(powerFrontLeft_requested - powerFrontLeft_current), abs(looptime.seconds() * maxRampPerSec)) * signum(powerFrontLeft_requested - powerFrontLeft_current);
            if (powerFrontLeft_current != powerFrontLeft_old) {
                powerFrontLeft_old = powerFrontLeft_current;
            }
        }
        if (powerFrontRight_requested == 0) {
            powerFrontRight_current = powerFrontRight_old = 0;
        } else {
            powerFrontRight_current += min(abs(powerFrontRight_requested - powerFrontRight_current), abs(looptime.seconds() * maxRampPerSec)) * signum(powerFrontRight_requested - powerFrontRight_current);
            if (powerFrontRight_current != powerFrontRight_old) {
                powerFrontRight_old = powerFrontRight_current;
            }
        }
        if (powerRearLeft_requested == 0) {
            powerRearLeft_current = powerRearLeft_old = 0;
        //    driveRearLeft.setPower(powerRearLeft_current);
        } else {
            powerRearLeft_current += min(abs(powerRearLeft_requested - powerRearLeft_current), abs(looptime.seconds() * maxRampPerSec)) * signum(powerRearLeft_requested - powerRearLeft_current);
            if (powerRearLeft_current != powerRearLeft_old) {
                powerRearLeft_old = powerRearLeft_current;
            }
        }
        if (powerRearRight_requested == 0) {
            powerRearRight_current = powerRearRight_old = 0;
       //     driveRearRight.setPower(powerRearRight_current);
        } else {
            powerRearRight_current += min(abs(powerRearRight_requested - powerRearRight_current), abs(looptime.seconds() * maxRampPerSec)) * signum(powerRearRight_requested - powerRearRight_current);
            if (powerRearRight_current != powerRearRight_old) {
                powerRearRight_old = powerRearRight_current;
            }
        }


        if(smartMode)
        {
            driveFrontLeft.setVelocity(powerFrontLeft_current*2300);
            driveFrontRight.setVelocity(powerFrontRight_current*2300);
            driveRearLeft.setVelocity(powerRearLeft_current*2300);
            driveRearRight.setVelocity(powerRearRight_current*2300);
        }
        else
        {
            driveFrontLeft.setPower(powerFrontLeft_current);
            driveFrontRight.setPower(powerFrontRight_current);
            driveRearLeft.setPower(powerRearLeft_current);
            driveRearRight.setPower(powerRearRight_current);
        }

        looptime.reset();
    }

    public void driveMotorPowers_direct(double frontLeft, double frontRight, double rearLeft, double rearRight) {
        driveFrontLeft.setPower(frontLeft);
        driveFrontRight.setPower(frontRight);
        driveRearLeft.setPower(rearLeft);
        driveRearRight.setPower(rearRight);
    }

    public void driveMotorPowers(double frontLeft, double frontRight, double rearLeft, double rearRight) {

        double maxabs = max(max(abs(frontLeft), abs(frontRight)), max(abs(rearLeft), abs(rearRight)));
        if (maxabs > maxDriveSpeed) {
            frontLeft = (frontLeft / maxabs) * maxDriveSpeed;
            frontRight = (frontRight / maxabs) * maxDriveSpeed;
            rearLeft = (rearLeft / maxabs) * maxDriveSpeed;
            rearRight = (rearRight / maxabs) * maxDriveSpeed;
        }

        frontLeft = limitSpeed(frontLeft);
        frontRight = limitSpeed(frontRight);
        rearLeft = limitSpeed(rearLeft);
        rearRight = limitSpeed(rearRight);

        powerFrontLeft_requested = (clip(abs(frontLeft), minDriveSpeed, 1) * signum(frontLeft));
        powerFrontRight_requested = (clip(abs(frontRight), minDriveSpeed, 1) * signum(frontRight));
        powerRearLeft_requested = (clip(abs(rearLeft), minDriveSpeed, 1) * signum(rearLeft));
        powerRearRight_requested = (clip(abs(rearRight), minDriveSpeed, 1) * signum(rearRight));
    }

    private double limitSpeed(double speed) {
        return clip(abs(speed), minDriveSpeed, 1) * signum(speed);
    }


    public void setRobotVelocity(double frontways, double sideways, double turn) {
        double FrontLeft = frontways + sideways + turn;
        double FrontRight = frontways - sideways - turn;
        double RearLeft = frontways - sideways + turn;
        double RearRight = frontways + sideways - turn;

        driveMotorPowers(FrontLeft, FrontRight, RearLeft, RearRight);
    }


}