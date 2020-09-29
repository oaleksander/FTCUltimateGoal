package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.sin;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.WoENmath.cosFromSin;

public class WoENodometry extends Thread
{
    HardwareMap hwMap           =  null;
    ExpansionHubMotor odometerYL, odometerYR, odometerX;
    RevBulkData bulkData;
    ExpansionHubEx expansionHub;
    public void run() {
        bulkData = expansionHub.getBulkInputData();
        double Y_old = (double)(bulkData.getMotorCurrentPosition(odometerYL)+bulkData.getMotorCurrentPosition(odometerYR))/2.0;
        double X_old = (double)bulkData.getMotorCurrentPosition(odometerX);
        float previousAngle = getRawAngle();
        integratedAngle += previousAngle;
        double worldAngle;
        while (opModeIsActive() && !stopOdometry) {
            bulkData = expansionHub.getBulkInputData();

            //Get Current Positions
            double currentY = (double)(bulkData.getMotorCurrentPosition(odometerYL)+bulkData.getMotorCurrentPosition(odometerYR))/2.0;
            double currentX = (double)bulkData.getMotorCurrentPosition(odometerX);
            double newWorldAngle = getAngle();

            double deltaAngle = newWorldAngle - worldAngle; ////

            double sinDeltaAngle = Math.sin(deltaAngle);
            double cosDeltaAngle = cosFromSin(sinDeltaAngle, deltaAngle);

            int deltaYodometer = currentY - Y_old;
            int deltaXodometer = currentX - X_old;

            double deltaPositionY = deltaYodometer;
            double deltaPositionX = deltaXodometer;

            if (deltaAngle != 0) {

                double sinDeltaAngle = sin(deltaAngle);
                double cosDeltaAngle = cosFromSin(sinDeltaAngle, deltaAngle);

                double yOdometerArcRadius = deltaYodometer / deltaAngle;
                double xOdometerArcRadius = deltaXodometer / deltaAngle;

                double yOdometerComponent = sinDeltaAngle * yOdometerArcRadius * cosDeltaAngle;
                double xOdometerComponent = sinDeltaAngle * xOdometerArcRadius * sinDeltaAngle;

                deltaPositionY = yOdometerComponent - xOdometerComponent
                deltaPositionX = yOdometerComponent + xOdometerComponent

            }

            double sinWorldAngle = sin(worldAngle);
            double cosWorldAngle = cosFromSin(sinWorldAngle, worldAngle);

            WorldPosition.y += deltaPositionY * cosWorldAngle - deltaPositionX * sinWorldAngle;
            WorldPosition.x += deltaPositionY * sinWorldAngle + deltaPositionX * cosWorldAngle;

            Y_old = currentY;
            X_old = currentX;
            worldAngle = newWorldAngle;
            sleep(10);
        }
        if (stopOdometry)
            stopOdometry = false;
    }

    //Odometry wheels
    private DcMotor encoderFrontLeft, encoderFrontRight, encoderRearLeft, encoderRearRight;
    private BNO055IMU imu;
    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    private int currentFrontLeft = 0, currentFrontRight = 0, currentRearLeft = 0, currentRearRight = 0;
    private float currentAngle = 0;
    private double robotEstimatedHeading;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0;
    private int previousFrontLeft = 0, previousFrontRight = 0, previousRearLeft = 0, previousRearRight = 0;
    private float previousAngle = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    private float referenceZeroAngle = 0;

    //Files to access the algorithm constants


    public WoENodometry(HardwareMap ahwMap) {
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
    public double returnXCoordinate() {
        return robotGlobalXCoordinatePosition;
    }

    /**
     * Returns the robot's global y coordinate
     *
     * @return global y coordinate
     */
    public double returnYCoordinate() {
        return robotGlobalYCoordinatePosition;
    }

    /**
     * Returns the robot's global orientation
     *
     * @return global orientation, in degrees
     */
    public double returnOrientation() {
        return Math.toDegrees(robotEstimatedHeading) % 360;
    }

    /**
     * Stops the position update thread
     */
    public void stop() {
        isRunning = false;
    }

    /**
     * to access IMU
     */
    private float getAngleRad() {
        float angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - referenceZeroAngle;
        if (angle > Math.PI) angle -= 2 * Math.PI;
        if (angle < Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while (isRunning) {
            globalCoordinatePositionUpdate();
            referenceZeroAngle = getAngleRad();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}