package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.misc.CommandSender;
import org.firstinspires.ftc.teamcode.misc.motorAccelerationLimiter;
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.jetbrains.annotations.NotNull;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.signum;

public class MecanumDrivetrain implements RobotModule, Drivetrain {



    /* Drivetrain hardware members. */
    DcMotorEx driveFrontLeft = null;
    DcMotorEx driveFrontRight = null;
    DcMotorEx driveRearLeft = null;
    DcMotorEx driveRearRight = null;

    /* Physical constants */
    private static final double wheelRadius = 9.8/2;
    private static final double strafingMultiplier = 1/0.8;
    private static final Vector2D wheelCenterOffset = new Vector2D(18.05253,15.20000);
    private static final double rotationDecrepancy = 1.0;
    private static final double forwardMultiplier = 1/wheelRadius;
    private static final double sidewaysMultiplier = forwardMultiplier*strafingMultiplier;
    private static final double turnMultiplier = (wheelCenterOffset.x+wheelCenterOffset.y)*rotationDecrepancy/wheelRadius;


    /* Motor parameters constatnts. */
    private static final PIDFCoefficients drivePIDFCoefficients = new PIDFCoefficients(15.00, 0.075, 15, 15.00);
    private static final double achieveableMaxRPMFraction = 0.9;
    private static final double tickPerRev = 480;
    private static final double gearing = 20;
    private static final double maxRPM = 300;
    public static final double theoreticalMaxSpeed = (maxRPM/60)*Math.PI*2;

    private static double maxMotorSpeed = achieveableMaxRPMFraction*theoreticalMaxSpeed;
    private static double minMotorSpeed = 0.05*theoreticalMaxSpeed;
    private double maxAcceleration = theoreticalMaxSpeed / 0.25;

    /* Motor controllers */
    private final motorAccelerationLimiter mFLProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveFrontLeft.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);
    private final motorAccelerationLimiter mFRProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveFrontRight.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);
    private final motorAccelerationLimiter mRLProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveRearLeft.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);
    private final motorAccelerationLimiter mRRProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveRearRight.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);



    private boolean smartMode = true;
    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerRearLeft = 0;
    private double powerRearRight = 0;
    private LinearOpMode opMode = null;

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initialize() {
        assignNames();
        setMotorDirections();
        setMotor0PowerBehaviors(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setMotorConfiguration(achieveableMaxRPMFraction,tickPerRev,gearing,maxRPM);
        try {
            setPIDFCoefficients(drivePIDFCoefficients);
        }catch(UnsupportedOperationException e)
        {
            opMode.telemetry.addData("Drivetrain PIDF error ", e.getMessage());
        }
        setSmartMode(true);
        setRobotVelocity(0, 0, 0);
    }

    public void setSmartMode(boolean SmartMode) {
        smartMode = SmartMode;
        if (smartMode) {
            setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
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

    private void setPIDFCoefficients(PIDFCoefficients pidfCoefficients)
    {
        driveFrontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        driveFrontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        driveRearLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        driveRearRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    private void setMotorConfiguration(double achieveableMaxRPMFraction, double tickPerRev, double gearing, double maxRPM)
    {
        setMotorConfiguration(driveFrontLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
        setMotorConfiguration(driveFrontRight, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
        setMotorConfiguration(driveRearLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
        setMotorConfiguration(driveRearRight, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
    }

    private void setMotorConfiguration(@NotNull DcMotorEx dcMotor, double achieveableMaxRPMFraction, double tickPerRev, double gearing, double maxRPM)
    {
        MotorConfigurationType motorConfigurationType = dcMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(achieveableMaxRPMFraction);
        motorConfigurationType.setTicksPerRev(tickPerRev);
        motorConfigurationType.setGearing(gearing);
        motorConfigurationType.setMaxRPM(maxRPM);
        dcMotor.setMotorType(motorConfigurationType);
    }

    private void resetEncoders() {
        setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMaxDriveSpeed(double value) {
        maxMotorSpeed = clip(abs(value), 0, theoreticalMaxSpeed);
    }

    public void setMinDriveSpeed(double value) {
        minMotorSpeed = clip(abs(value), 0, theoreticalMaxSpeed);
    }

    public void update() {
        mFLProfiler.setVelocity(powerFrontLeft);
        mFRProfiler.setVelocity(powerFrontRight);
        mRLProfiler.setVelocity(powerRearLeft);
        mRRProfiler.setVelocity(powerRearRight);
    }

    public void driveMotorPowers_direct(double frontLeft, double frontRight, double rearLeft, double rearRight) {
        driveFrontLeft.setPower(frontLeft);
        driveFrontRight.setPower(frontRight);
        driveRearLeft.setPower(rearLeft);
        driveRearRight.setPower(rearRight);
    }

    public void driveMotorPowers(double frontLeft, double frontRight, double rearLeft, double rearRight) {

        double maxabs = max(max(abs(frontLeft), abs(frontRight)), max(abs(rearLeft), abs(rearRight)));
        if (maxabs > maxMotorSpeed) {
            maxabs = maxabs/ maxMotorSpeed;
            frontLeft = (frontLeft / maxabs);
            frontRight = (frontRight / maxabs);
            rearLeft = (rearLeft / maxabs);
            rearRight = (rearRight / maxabs);
        }

        powerFrontLeft = limitSpeed(frontLeft);
        powerFrontRight = limitSpeed(frontRight);
        powerRearLeft = limitSpeed(rearLeft);
        powerRearRight = limitSpeed(rearRight);
    }

    private double limitSpeed(double speed) {
        return clip(abs(speed), minMotorSpeed, maxMotorSpeed) * signum(speed);
    }

    public Vector3D getMaxRobotVelocity() {
        return new Vector3D(maxMotorSpeed/forwardMultiplier,maxMotorSpeed/forwardMultiplier,maxMotorSpeed/turnMultiplier);
    }

    public void setRobotVelocity(double frontways, double sideways, double turn) {

        frontways *= forwardMultiplier;
        sideways *= sidewaysMultiplier;
        turn *= turnMultiplier;

        double FrontLeft = frontways + sideways + turn;
        double FrontRight = frontways - sideways - turn;
        double RearLeft = frontways - sideways + turn;
        double RearRight = frontways + sideways - turn;

        driveMotorPowers(FrontLeft, FrontRight, RearLeft, RearRight);
    }


}