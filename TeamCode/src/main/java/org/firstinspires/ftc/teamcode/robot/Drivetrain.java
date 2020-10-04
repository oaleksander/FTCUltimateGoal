package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Vector;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.rint;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap;

public class Drivetrain {

    /* Drivetrain hardware members. */
    public static ExpansionHubEx expansionHub1 = null;
    public static DcMotorEx driveFrontLeft = null;
    public static DcMotorEx driveFrontRight = null;
    public static DcMotorEx driveRearLeft = null;
    public static DcMotorEx driveRearRight = null;

    public static ExpansionHubEx expansionHub2 = null;
    public static ExpansionHubMotor odometerYL = null;
    public static ExpansionHubMotor odometerYR = null;
    public static ExpansionHubMotor odometerX = null;

    public static BNO055IMU imuLeft;
    public static BNO055IMU imuRight;

    /* Drivetrain constatnts. */

    public static final double odometryWheelDiameterCm = 4.8;
    public static final double odometryCountsPerCM = (1440) / (odometryWheelDiameterCm * PI);
    public static final double odometryCMPerCounts = (odometryWheelDiameterCm * PI) / 1440;
    public static final float odometerYcenterOffsetCm = 13;
    public static final float odometerXcenterOffsetCm = 2;
    public float imuLeftAngleOffset = 0;
    public float imuRightAngleOffset = 0;

    public static double maxDriveSpeed = 1;
    public static double minDriveSpeed = 0.1;

    public Drivetrain() {
    }

    public void initialize() {
        assignNames();
        setMotorDirections();
        setMotor0PowerBehaviors();
        stopMotors();
    }

    private void assignNames() {
        driveFrontLeft = WoENrobot.getInstance().opMode.hardwareMap.get(DcMotorEx.class, "driveFrontLeft");
        driveFrontRight = WoENrobot.getInstance().opMode.hardwareMap.get(DcMotorEx.class, "driveFrontRight");
        driveRearLeft = WoENrobot.getInstance().opMode.hardwareMap.get(DcMotorEx.class, "driveRearLeft");
        driveRearRight = WoENrobot.getInstance().opMode.hardwareMap.get(DcMotorEx.class, "driveRearRight");
    }

    private void setMotorDirections() {
        driveFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        driveFrontRight.setDirection(DcMotorEx.Direction.FORWARD);
        driveRearLeft.setDirection(DcMotorEx.Direction.REVERSE);
        driveRearRight.setDirection(DcMotorEx.Direction.FORWARD);

    }

    private void setMotor0PowerBehaviors() {
        driveFrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        driveFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        driveRearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        driveRearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    private void resetEncoders()
    {
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMaxDriveSpeed(double value) {
        maxDriveSpeed = clip(abs(value), 0, 1);
    }

    public void setMinDriveSpeed(double value) {
        minDriveSpeed = clip(abs(value), 0, 1);
    }

    public void driveMotorPowers(double frontLeft, double frontRight, double rearLeft, double rearRight) {
        driveFrontLeft.setPower(clip(abs(frontLeft), minDriveSpeed, maxDriveSpeed) * signum(frontLeft));
        driveFrontRight.setPower(clip(abs(frontRight), minDriveSpeed, maxDriveSpeed) * signum(frontRight));
        driveRearLeft.setPower(clip(abs(rearLeft), minDriveSpeed, maxDriveSpeed) * signum(rearLeft));
        driveRearRight.setPower(clip(abs(rearRight), minDriveSpeed, maxDriveSpeed) * signum(rearRight));
    }

    public void stopMotors() {
        driveMotorPowers(0, 0, 0, 0);
    }

    private void tankMove(double speedL, double speedR) {
        speedL = Range.clip(speedL, -1.0, 1.0);
        speedR = Range.clip(speedR, -1.0, 1.0);

        driveMotorPowers(speedL, speedR, speedL, speedR);
    }

    public void tankMove(double speed) {
        tankMove(speed, speed);
    }

    public void tankTurn(double speed) {
        tankMove(speed, -speed);
    }

    void holonomicMove(double frontways, double sideways) {

        double LFRR = frontways + sideways;
        double RFLR = frontways - sideways;

        if (abs(LFRR) > 1.0 || abs(RFLR) > 1.0) {
            LFRR /= max(abs(LFRR), abs(RFLR));
            RFLR /= max(abs(LFRR), abs(RFLR));
        }

        driveMotorPowers(LFRR, RFLR, RFLR, LFRR);
    }

    public void holonomicMove(double frontways, double sideways, double turn) {
        double FrontLeft = frontways + sideways + turn;
        double FrontRight = frontways - sideways - turn;
        double RearLeft = frontways - sideways + turn;
        double RearRight = frontways + sideways - turn;

        if (abs(FrontLeft) > 1.0 || abs(FrontRight) > 1.0 || abs(RearLeft) > 1.0 || abs(RearRight) > 1.0) {
            double maxabs = max(max(abs(FrontLeft), abs(FrontRight)), max(abs(RearLeft), abs(RearRight)));
            FrontLeft /= maxabs;
            FrontRight /= maxabs;
            RearLeft /= maxabs;
            RearRight /= maxabs;
        }

        driveMotorPowers(FrontLeft, FrontRight, RearLeft, RearRight);
    }

    void holonomicMovePolar(double heading, double speed, double turn) {
        turn = Range.clip(turn, -1.0, 1.0);
        speed = Range.clip(speed, -1.0, 1.0);
        double frontways = speed * cos(heading);
        double sideways = speed * sin(heading);

        holonomicMove(frontways, sideways, turn);
    }

    void holonomicMovePolar(double heading, double speed) {
        speed = Range.clip(speed, -1.0, 1.0);
        holonomicMove(speed * cos(heading), speed * sin(heading));
    }

    public void holonomicMove(@NotNull Pose2D move) {
        holonomicMove(move.y, move.x, move.heading);
    }

    public void holonomicMoveFC(@NotNull Pose2D move) {
        Vector2D coordinates = new Vector2D(move.x, move.y).rotated(-WoENrobot.odometry.getRobotCoordinates().heading);
        holonomicMove(coordinates.x, coordinates.y, move.heading);
    }

    public static final double kP_distance = 0.0011, kD_distance = 0.000022;
    public static final double minError_distance = 5 * odometryCountsPerCM;

    public static final double kP_angle = 0.0, kD_angle = 0;
    public static final double minError_angle = Math.toRadians(3);

    public void Pos(@NotNull Pose2D target) {

        Pose2D error = target.substract(WoENrobot.odometry.getRobotCoordinates());
        Pose2D errold = error;
        double distanceError = error.radius();

        ElapsedTime looptime = new ElapsedTime();
        while (WoENrobot.getInstance().opMode.opModeIsActive() && (distanceError > minError_distance || abs(error.heading) > minError_angle)) {

            error = target.substract(WoENrobot.odometry.getRobotCoordinates());

            Pose2D differr = (error.substract(errold)).divideByDouble(looptime.seconds());
            looptime.reset();

            Pose2D control = new Pose2D(
                    error.x * kP_distance + differr.x * kD_distance,
                    error.y * kP_distance + differr.y * kD_distance,
                    differr.heading * kP_angle + differr.heading * kP_angle);

            //holonomicMoveFC(control);

            holonomicMove(0, 0, 0);
            WoENrobot.getInstance().opMode.telemetry.addData("y", control.y);
            WoENrobot.getInstance().opMode.telemetry.addData("x", control.x);
            WoENrobot.getInstance().opMode.telemetry.addData("a", control.heading);
            WoENrobot.getInstance().opMode.telemetry.addData("ae", toDegrees(error.heading));
            WoENrobot.getInstance().opMode.telemetry.addData("dist", odometryCMPerCounts * distanceError);
            WoENrobot.getInstance().opMode.telemetry.update();

            errold = error;
            distanceError = error.radius();
        }
    }


}