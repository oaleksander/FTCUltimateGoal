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