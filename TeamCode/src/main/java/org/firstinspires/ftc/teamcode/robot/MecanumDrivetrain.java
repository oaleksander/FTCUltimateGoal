package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.jetbrains.annotations.NotNull;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.signum;

public class MecanumDrivetrain extends MultithreadRobotModule implements Drivetrain {


    /* Physical constants */
    private static final double wheelRadius = 9.8 / 2;
    private static final double gearRatio = 1;
    private static final Vector2D wheelCenterOffset = new Vector2D(18.05253, 15.20000);
    private static final double forwardMultiplier = (1 / wheelRadius)/gearRatio;
    private static double sidewaysMultiplier = forwardMultiplier * DrivetrainConfig.strafingMultiplier;
    private static double turnMultiplier = (wheelCenterOffset.x + wheelCenterOffset.y) * DrivetrainConfig.rotationDecrepancy / wheelRadius;

    /* Motor parameters constatnts. */
    @Config
    static class DrivetrainConfig {
        public static double achieveableMaxRPMFraction = 0.885;
        public static double achieveableMinRPMFraction = 0.045;
        public static double strafingMultiplier = 1.35;
        public static double rotationDecrepancy = 1;
        public static double secondsToAccelerate = 0.33;
        public static double kP = 26;
        public static double kD = 0;
        public static double kI = 0.1;
        public static double kF = 15.10;
        public static double kF_referenceVoltage = 13;
    }

    private static final double tickPerRev = 480;
    private static final double gearing = 20;
    private static final double maxRPM = 300;
    public static final double theoreticalMaxSpeed = (maxRPM / 60) * Math.PI * 2;
    private static double maxMotorSpeed = DrivetrainConfig.achieveableMaxRPMFraction * theoreticalMaxSpeed;
    private static double minMotorSpeed = DrivetrainConfig.achieveableMinRPMFraction * theoreticalMaxSpeed; //http://b1-srv-kms-1.sch239.net:8239
    private double maxAcceleration = theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate;
    /* Drivetrain hardware members. */
    public static DcMotorEx driveFrontLeft = null;
    /* Motor controllers */
    private final motorAccelerationLimiter mFLProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveFrontLeft.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);
    public static DcMotorEx driveFrontRight = null;
    private final motorAccelerationLimiter mFRProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveFrontRight.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);
    public static DcMotorEx driveRearLeft = null;
    private final motorAccelerationLimiter mRLProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveRearLeft.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);
    public static DcMotorEx driveRearRight = null;
    private final motorAccelerationLimiter mRRProfiler = new motorAccelerationLimiter(new CommandSender(v -> driveRearRight.setVelocity(v, AngleUnit.RADIANS))::send, maxAcceleration);


    private boolean smartMode = false;
    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerRearLeft = 0;
    private double powerRearRight = 0;

    public void initialize() {
        assignNames();
        setMotorDirections();
        maxMotorSpeed = DrivetrainConfig.achieveableMaxRPMFraction * theoreticalMaxSpeed;
        minMotorSpeed = DrivetrainConfig.achieveableMinRPMFraction * theoreticalMaxSpeed;
        sidewaysMultiplier = forwardMultiplier * DrivetrainConfig.strafingMultiplier;
        maxAcceleration = theoreticalMaxSpeed / DrivetrainConfig.secondsToAccelerate;
        turnMultiplier = (wheelCenterOffset.x + wheelCenterOffset.y) * DrivetrainConfig.rotationDecrepancy / wheelRadius;
        setMotor0PowerBehaviors(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setMotorConfiguration(DrivetrainConfig.achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
        try {
            setPIDFCoefficients(new PIDFCoefficients(DrivetrainConfig.kP, DrivetrainConfig.kD, DrivetrainConfig.kI, DrivetrainConfig.kF * DrivetrainConfig.kF_referenceVoltage / opMode.hardwareMap.voltageSensor.iterator().next().getVoltage()));
        } catch (UnsupportedOperationException e) {
            opMode.telemetry.addData("Drivetrain PIDF error ", e.getMessage());
        }
        setSmartMode(true);
        setRobotVelocity(0, 0, 0);
    }

    public void setSmartMode(boolean SmartMode) {
        smartMode = SmartMode;
        setMotorMode(SmartMode ? DcMotorEx.RunMode.RUN_USING_ENCODER : DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void assignNames() {
        driveFrontLeft = WoENHardware.INSTANCE.getDriveFrontLeft();
        driveFrontRight = WoENHardware.INSTANCE.getDriveFrontRight();
        driveRearLeft = WoENHardware.INSTANCE.getDriveRearLeft();
        driveRearRight = WoENHardware.INSTANCE.getDriveRearRight();
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

    private void setPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        driveFrontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        driveFrontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        driveRearLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        driveRearRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    private void setMotorConfiguration(double achieveableMaxRPMFraction, double tickPerRev, double gearing, double maxRPM) {
        setMotorConfiguration(driveFrontLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
        setMotorConfiguration(driveFrontRight, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
        setMotorConfiguration(driveRearLeft, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
        setMotorConfiguration(driveRearRight, achieveableMaxRPMFraction, tickPerRev, gearing, maxRPM);
    }

    private void setMotorConfiguration(@NotNull DcMotorEx dcMotor, double achieveableMaxRPMFraction, double tickPerRev, double gearing, double maxRPM) {
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

    public void updateControlHub() {
        if (smartMode) {
            mFLProfiler.setVelocity(powerFrontLeft);
            mFRProfiler.setVelocity(powerFrontRight);
            mRLProfiler.setVelocity(powerRearLeft);
            mRRProfiler.setVelocity(powerRearRight);
        } else
            driveMotorPowers_direct(powerFrontLeft / maxMotorSpeed,
                    powerFrontRight / maxMotorSpeed,
                    powerRearLeft / maxMotorSpeed,
                    powerRearRight / maxMotorSpeed);
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
            maxabs = maxabs / maxMotorSpeed;
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

    public Vector3D getMaxVelocity() {
        return new Vector3D(maxMotorSpeed / forwardMultiplier, maxMotorSpeed / forwardMultiplier, maxMotorSpeed / turnMultiplier);
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