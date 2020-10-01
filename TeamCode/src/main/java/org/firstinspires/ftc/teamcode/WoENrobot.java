package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.WoENmath.cosFromSin;

public class WoENrobot extends LinearOpMode {


    /* Public OpMode members. */
    public ElapsedTime Runtime = new ElapsedTime();
    public static boolean robotIsInitialized = false;
    public static DcMotor driveFrontLeft = null;
    public static DcMotor driveFrontRight = null;
    public static DcMotor driveRearLeft = null;
    public static DcMotor driveRearRight = null;

    public static DcMotor liftL = null;
    public static DcMotor liftR = null;

    public static CRServo slideFront = null;
    public static CRServo slideRear = null;

    public static Servo brickGrabberFront = null;
    public static Servo brickGrabberRear = null;

    public static Servo foundationHookL = null;
    public static Servo foundationHookR = null;

    public static DcMotor odometerYintakeR = null;
    public static DcMotor odometerXintakeL = null;

    public static BNO055IMU imuLeft;
    public static BNO055IMU imuRight;

    public static DigitalChannel limitSwitchFront;
    public static DigitalChannel limitSwitchRear;

    public static int cameraMonitorViewId;

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;

    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    public static int randomvariable;

    /* local OpMode members. */
    private ElapsedTime period = new ElapsedTime();


    public static final double odometryWheelDiameterCm = 4.8;
    public static final double odometryCountsPerCM = (1440) / (odometryWheelDiameterCm * PI);
    public static final double odometryCMPerCounts = (odometryWheelDiameterCm * PI) / 1440;
    public static final float odometerYcenterOffset = 13;
    public static double maxDriveSpeed = 1;
    public static double minDriveSpeed = 0.1;
    public static boolean drive0Brake = false;
    // public static final double odometerXcenterOffset = 0;

    public static final int liftEncoderMin = 0;
    public static final int liftEncoderMax = 4200;

    public static final double brickGrabberFront_Open = 0.39;
    public static final double brickGrabberFront_Close = 0.00;
    public static final double brickGrabberRear_Open = 0.34;
    public static final double brickGrabberRear_Close = 0.00;
    public static final double foundationHookL_Open = 0.03;
    public static final double foundationHookL_Close = 0.53;
    public static final double foundationHookR_Open = 0.97;
    public static final double foundationHookR_Close = 0.47;

    public float imuLeftAngleOffset = 0;
    public float imuRightAngleOffset = 0;

    /* Constructor */
    @Override
    public void runOpMode() {
    }


    private double robotEstimatedHeading;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0;
    private Point WorldPosition;
    private float integratedAngle = 0;

    private boolean stopOdometry = false;
    private boolean stopIntakeControl = false;
    private boolean stopManipulatorControl = false;
    private boolean stopSlideControl = false;
    private boolean stopLiftControl = false;

    public static final double kP_distance = 0.0011, kD_distance = 0.000022;
    public static final double minError_distance = 5 * odometryCountsPerCM;

    public static final double kP_angle = 0.0, kD_angle = 0;
    public static final double minError_angle = Math.toRadians(3);

    public void Pos(int y_target, int x_target, double heading_target) {
        y_target *= odometryCountsPerCM;
        x_target *= odometryCountsPerCM;
        heading_target = angleTransformPI(toRadians(heading_target));
        ElapsedTime looptime = new ElapsedTime();
        double errold_y = y_target - robotGlobalYCoordinatePosition;
        double errold_x = x_target - robotGlobalXCoordinatePosition;
        double err_y = errold_y;
        double err_x = errold_x;
        double distanceError = Math.hypot(err_y, err_x);
        double angleError = angleTransformPI(heading_target - robotEstimatedHeading);
        double differr_angle;
        double errold_angle = angleError;


        while (opModeIsActive() && (distanceError > minError_distance || abs(angleError) > minError_angle)) {

            err_y = y_target - robotGlobalYCoordinatePosition;
            err_x = x_target - robotGlobalXCoordinatePosition;
            angleError = angleTransformPI(heading_target - robotEstimatedHeading);


            differr_angle = (angleError - errold_angle) / looptime.seconds();
            double differr_y = (err_y - errold_y) / looptime.seconds();
            double differr_x = (err_x - errold_x) / looptime.seconds();
            looptime.reset();
            errold_angle = angleError;
            errold_y = err_y;
            errold_x = err_x;

            double relativeY = err_y * kP_distance + differr_y * kD_distance;
            double relativeX = err_x * kP_distance + differr_x * kD_distance;

            double cosa = cos(robotEstimatedHeading);
            double sina = sin(robotEstimatedHeading);

            double controlY = relativeX * sina + relativeY * cosa;
            double controlX = relativeX * cosa - relativeY * sina;
            double controlAngle = angleError * kP_angle + differr_angle * kP_angle;

            omniMoveYX(0, 0, 0);

            telemetry.addData("y", controlY);
            telemetry.addData("x", controlX);
            telemetry.addData("a", controlAngle);
            telemetry.addData("ae", toDegrees(angleError));

            telemetry.addData("dist", odometryCMPerCounts * distanceError);
            telemetry.update();

            distanceError = Math.hypot(err_y, err_x);
        }
    }


    boolean doGrabStone = false;

    Thread autoManipulator = new Thread(new Runnable() {
        public void run() {
            while (opModeIsActive() && !stopManipulatorControl) {
                if (doGrabStone) {
                    intakeMotorPowers(0, 0);
                    grabBrick(true, true);
                    if (!limitSwitchRear.getState()) {

                        slideMotorPowers(-1);
                    } else {
                        slideMotorPowers(0);
                    }

                } else {
                    if (!limitSwitchFront.getState()) {
                        grabBrick(false, true);
                        slideMotorPowers(1);
                        intakeMotorPowers(0, 0);
                    } else {
                        grabBrick(true, false);
                        slideMotorPowers(0);
                        intakeMotorPowers(-1, -1);
                    }
                }
            }
            if (stopManipulatorControl)
                stopManipulatorControl = false;
        }
    });


    boolean doSlideForward = false;

    Thread slideControl = new Thread(new Runnable() {
        public void run() {
            while (opModeIsActive() && !stopSlideControl) {
                if (doSlideForward) {
                    if (!limitSwitchRear.getState()) {
                        slideMotorPowers(-1);
                    } else {
                        slideMotorPowers(0);
                    }

                } else {
                    if (!limitSwitchFront.getState()) {
                        slideMotorPowers(1);
                    } else {
                        slideMotorPowers(0);
                    }
                }
            }
            if (stopSlideControl)
                stopSlideControl = false;
        }
    });

    public static int liftTarget = 0;

    public void liftManipulator(int target_cm) {
        if (!liftControl.isAlive()) {
            liftControl.start();
        }
        liftTarget = target_cm;
    }

    Thread liftControl = new Thread(new Runnable() {
        public void run() {
            while (opModeIsActive() && !stopLiftControl) {


                sleep(10);
            }
            stopLiftMotors();
            if (stopLiftControl)
                stopLiftControl = false;
        }
    });


    public double returnXCoordinate() {
        return robotGlobalXCoordinatePosition * odometryCMPerCounts;
    }

    /**
     * Returns the robot's global y coordinate
     *
     * @return global y coordinate
     */
    public double returnYCoordinate() {
        return robotGlobalYCoordinatePosition * odometryCMPerCounts;
    }

    /**
     * Returns the robot's global orientation
     *
     * @return global orientation, in degrees
     */
    public double returnOrientation() {
        return Math.toDegrees(robotEstimatedHeading);
    }

    public double returnIncrementalOrientation() {
        return Math.toDegrees(integratedAngle);
    }

    public void startRobot() {
        waitForStart();
        Runtime.reset();
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    public void initRobot() {
        if (!robotIsInitialized) {
            forceInitRobot();
            telemetry.addData("Status", "Initialization successful");
            telemetry.update();
        } else {
            stopAllMotors();
            setMotor0PowerBehaviors();
            setMotorDirections();
            telemetry.addData("Status", "Already initialized, ready");
            telemetry.update();
        }
    }

    private static final boolean defaultNames = true;

    public void forceInitRobot() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        if (defaultNames) {

            imuLeft = hardwareMap.get(BNO055IMU.class, "imuLeft");
            imuRight = hardwareMap.get(BNO055IMU.class, "imuRight");

            driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
            driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
            driveRearLeft = hardwareMap.get(DcMotor.class, "driveRearLeft");
            driveRearRight = hardwareMap.get(DcMotor.class, "driveRearRight");

            liftL = hardwareMap.get(DcMotor.class, "liftL");
            liftR = hardwareMap.get(DcMotor.class, "liftR");

            slideFront = hardwareMap.get(CRServo.class, "slideFront");
            slideRear = hardwareMap.get(CRServo.class, "slideRear");

            brickGrabberFront = hardwareMap.get(Servo.class, "brickGrabberFront");
            brickGrabberRear = hardwareMap.get(Servo.class, "brickGrabberRear");

            foundationHookL = hardwareMap.get(Servo.class, "foundationHookL");
            foundationHookR = hardwareMap.get(Servo.class, "foundationHookR");

            odometerYintakeR = hardwareMap.get(DcMotor.class, "odometerYintakeR");
            odometerXintakeL = hardwareMap.get(DcMotor.class, "odometerXintakeL");

            limitSwitchFront = hardwareMap.get(DigitalChannel.class, "limitSwitchFront");
            limitSwitchRear = hardwareMap.get(DigitalChannel.class, "limitSwitchRear");

        } else {
            imuLeft = hardwareMap.get(BNO055IMU.class, "imuLeft");
            imuRight = hardwareMap.get(BNO055IMU.class, "imuRight");

            driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
            driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
            driveRearLeft = hardwareMap.get(DcMotor.class, "driveRearLeft");
            driveRearRight = hardwareMap.get(DcMotor.class, "driveRearRight");

            liftL = hardwareMap.get(DcMotor.class, "liftL");
            liftR = hardwareMap.get(DcMotor.class, "liftR");

            slideFront = hardwareMap.get(CRServo.class, "slideFront");
            slideRear = hardwareMap.get(CRServo.class, "slideRear");

            brickGrabberFront = hardwareMap.get(Servo.class, "brickGrabberFront");
            brickGrabberRear = hardwareMap.get(Servo.class, "brickGrabberRear");

            foundationHookL = hardwareMap.get(Servo.class, "foundationHookL");
            foundationHookR = hardwareMap.get(Servo.class, "foundationHookR");

            odometerYintakeR = hardwareMap.get(DcMotor.class, "odometerYintakeR");
            odometerXintakeL = hardwareMap.get(DcMotor.class, "odometerXintakeL");

            limitSwitchFront = hardwareMap.get(DigitalChannel.class, "limitSwitchFront");
            limitSwitchRear = hardwareMap.get(DigitalChannel.class, "limitSwitchRear");
        }

        limitSwitchFront.setMode(DigitalChannel.Mode.INPUT);
        limitSwitchRear.setMode(DigitalChannel.Mode.INPUT);

        setMotor0PowerBehaviors();
        setMotorDirections();

        initializeGyroscopes();

        grabBrick(true, false);
        grabFoundation(false);

        stopAllMotors();

        resetOdometryEncoders();
        resetLiftEncoders();

        robotIsInitialized = true;
        telemetry.addData("Status", "Force initialized");
        telemetry.update();
    }

    public void setMotorDirections() {
        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveRearLeft.setDirection(DcMotor.Direction.FORWARD);
        driveRearRight.setDirection(DcMotor.Direction.REVERSE);

        liftL.setDirection(DcMotor.Direction.FORWARD);
        liftR.setDirection(DcMotor.Direction.REVERSE);

        slideFront.setDirection(DcMotor.Direction.FORWARD);
        slideRear.setDirection(DcMotor.Direction.REVERSE);

        odometerYintakeR.setDirection(DcMotor.Direction.REVERSE);
        odometerXintakeL.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setMotor0PowerBehaviors() {
        if (drive0Brake) {
            driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            driveRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            driveRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }

        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odometerYintakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        odometerXintakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void initializeGyroscopes() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imuLeft.initialize(parameters);
        imuRight.initialize(parameters);

        imuLeftAngleOffset = -imuLeft.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        imuRightAngleOffset = -imuRight.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    float getAngle() {
        float leftAngle = radiansWrap(-imuLeft.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - imuLeftAngleOffset);
        float rightAngle = radiansWrap(-imuRight.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - imuRightAngleOffset);
        if (abs(leftAngle - rightAngle) > Math.PI)
            rightAngle += 2 * PI * signum(leftAngle - rightAngle);
        return (leftAngle + rightAngle) / 2;
    }

    private float radiansWrap(float angle) {
        if (angle > Math.PI) angle -= 2 * Math.PI;
        if (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void resetOdometryEncoders() {
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void resetLiftEncoders() {
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void resetDriveEncoders() {
        odometerYintakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerXintakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odometerYintakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometerXintakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopAllMotors() {
        stopMoveMotors();
        stopLiftMotors();
        stopIntakeMotors();
        stopSlideMotors();
    }

    public void grabBrick(boolean front, boolean rear) {
        brickGrabberFront.setPosition(front ? brickGrabberFront_Close : brickGrabberFront_Open);
        brickGrabberRear.setPosition(rear ? brickGrabberRear_Close : brickGrabberRear_Open);
    }

    public void grabFoundation(boolean grab) {
        foundationHookL.setPosition(grab ? foundationHookL_Close : foundationHookL_Open);
        foundationHookR.setPosition(grab ? foundationHookR_Close : foundationHookR_Open);
    }

    void stopMoveMotors() {
        driveMotorPowers(0, 0, 0, 0);
    }

    private void driveMotorPowers(double frontLeft, double frontRight, double rearLeft, double rearRight) {
        driveFrontLeft.setPower(clip(abs(frontLeft), minDriveSpeed, maxDriveSpeed) * signum(frontLeft));
        driveFrontRight.setPower(clip(abs(frontRight), minDriveSpeed, maxDriveSpeed) * signum(frontRight));
        driveRearLeft.setPower(clip(abs(rearLeft), minDriveSpeed, maxDriveSpeed) * signum(rearLeft));
        driveRearRight.setPower(clip(abs(rearRight), minDriveSpeed, maxDriveSpeed) * signum(rearRight));
    }

    private void stopLiftMotors() {
        liftMotorPowers(0, 0);
    }

    public void liftMotorPowers(double left, double right) {
        liftL.setPower(left);
        liftR.setPower(right);
    }

    private void stopIntakeMotors() {
        intakeMotorPowers(0, 0);
    }

    public void intakeMotorPowers(double powerL, double powerR) {
        odometerYintakeR.setPower(powerR);
        odometerXintakeL.setPower(powerL);
    }

    private void stopSlideMotors() {
        slideMotorPowers(0);
    }

    private void slideMotorPowers(double power) {
        slideFront.setPower(power);
        slideRear.setPower(power);
    }

    void tankTurn(double speed) {
        tankMove(speed, -speed);
    }

    void tankMove(double speed) {
        tankMove(speed, speed);
    }


    private void tankMove(double speedL, double speedR) {
        speedL = Range.clip(speedL, -1.0, 1.0);
        speedR = Range.clip(speedR, -1.0, 1.0);

        driveMotorPowers(speedL, speedR, speedL, speedR);
    }

    void omniMove(double heading, double speed, double turn) {
        heading = Math.toRadians(heading);
        turn = Range.clip(turn, -1.0, 1.0);
        speed = Range.clip(speed, -1.0, 1.0);
        double frontways = speed * cos(heading);
        double sideways = speed * sin(heading);

        omniMoveYX(frontways, sideways, turn);
    }

    void omniMove(double heading, double speed) {
        heading = Math.toRadians(heading);
        speed = Range.clip(speed, -1.0, 1.0);
        omniMoveYX(speed * cos(heading), speed * sin(heading));
    }

    void omniMoveYX(double frontways, double sideways) {

        double LFRR = frontways + sideways;
        double RFLR = frontways - sideways;

        if (abs(LFRR) > 1.0 || abs(RFLR) > 1.0) {
            LFRR /= max(abs(LFRR), abs(RFLR));
            RFLR /= max(abs(LFRR), abs(RFLR));
        }

        driveMotorPowers(LFRR, RFLR, RFLR, LFRR);

    }

    void omniMoveYX(double frontways, double sideways, double turn) {


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

    public void setMaxDriveSpeed(double value) {
        maxDriveSpeed = clip(abs(value), 0, 1);
    }

    public void setMinDriveSpeed(double value) {
        minDriveSpeed = clip(abs(value), 0, 1);
    }

    public static double angleTransformPI(double angle) {
        while (angle > PI) angle -= PI * 2;
        while (angle < -PI) angle += PI * 2;
        return angle;
    }


    Odometry odometry = new Odometry();

    public class Odometry extends Thread
    {
        HardwareMap hwMap = null;
        ExpansionHubMotor odometerYL, odometerYR, odometerX;
        RevBulkData bulkData;
        ExpansionHubEx expansionHub;
        Point worldPosition;
        double worldHeading;

        ElapsedTime uptime;

        private boolean doStop = false;

        public synchronized void doStop() {
            this.doStop = true;
        }

        private synchronized boolean keepRunning() {
            return !this.doStop;
        }

        @Override
        public void run() {
            uptime.reset();
            bulkData = expansionHub.getBulkInputData();
            double Y_old = (double) (bulkData.getMotorCurrentPosition(odometerYL) + bulkData.getMotorCurrentPosition(odometerYR)) / 2.0;
            double X_old = (double) bulkData.getMotorCurrentPosition(odometerX);
            while (opModeIsActive() && keepRunning()) {


                bulkData = expansionHub.getBulkInputData();

                //Get Current Positions
                double currentY = (double) (bulkData.getMotorCurrentPosition(odometerYL) + bulkData.getMotorCurrentPosition(odometerYR)) / 2.0;
                double currentX = (double) bulkData.getMotorCurrentPosition(odometerX);
                double newWorldHeading = 0;

                double deltaAngle = newWorldHeading - worldHeading; ////

                double deltaYodometer = currentY - Y_old;
                double deltaXodometer = currentX - X_old;

                double deltaPositionY = deltaYodometer;
                double deltaPositionX = deltaXodometer;

                if (deltaAngle != 0) {


                    double sinDeltaAngle = sin(deltaAngle);
                    double cosDeltaAngle = cosFromSin(sinDeltaAngle, deltaAngle);

                    double yOdometerArcRadius = deltaYodometer / deltaAngle;
                    double xOdometerArcRadius = deltaXodometer / deltaAngle;

                    double yOdometerComponent = sinDeltaAngle * yOdometerArcRadius * cosDeltaAngle;
                    double xOdometerComponent = sinDeltaAngle * xOdometerArcRadius * sinDeltaAngle;

                    deltaPositionY = yOdometerComponent - xOdometerComponent;
                    deltaPositionX = yOdometerComponent + xOdometerComponent;

                }

                double sinWorldAngle = sin(worldHeading);
                double cosWorldAngle = cosFromSin(sinWorldAngle, worldHeading);

                worldPosition.y += deltaPositionY * cosWorldAngle - deltaPositionX * sinWorldAngle;
                worldPosition.x += deltaPositionY * sinWorldAngle + deltaPositionX * cosWorldAngle;

                Y_old = currentY;
                X_old = currentX;
                worldHeading = newWorldHeading;

                try {
                    Thread.sleep(0);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
        }

        public Odometry(HardwareMap ahwMap) {
            hwMap = ahwMap;
            expansionHub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 1");
            odometerYL = (ExpansionHubMotor) hardwareMap.dcMotor.get("odometerYLauncherL");
            odometerYR = (ExpansionHubMotor) hardwareMap.dcMotor.get("odometerYLauncherR");
            odometerX  = (ExpansionHubMotor) hardwareMap.dcMotor.get("odometerXRingIntake");
        }

        /**
         * Returns the robot's global x coordinate
         *
         * @return global x coordinate
         */
        public Point getRobotCoordinate() {
            Point centeredPoint = worldPosition;
            centeredPoint.y += 0;
            centeredPoint.x += 0;
            return centeredPoint;
        }

        /**
         * Returns the robot's global orientation
         *
         * @return global orientation
         */

        private double getRobotHeading(AngleUnit unit) {
            double angle = 0;
            if (unit == AngleUnit.DEGREES)
                angle = Math.toDegrees(angle);
            return angle;
        }
    }

}

