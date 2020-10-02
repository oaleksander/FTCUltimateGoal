package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.math.MathUtil;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.math.MathUtil.cosFromSin;


public class Odometry extends Thread
{
    ExpansionHubMotor odometerYL, odometerYR, odometerX;
    ExpansionHubEx expansionHub;
    RevBulkData bulkData;
    Pose2D worldPosition;

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
        while (keepRunning()) {


            bulkData = expansionHub.getBulkInputData();

            //Get Current Positions
            double currentY = (double) (bulkData.getMotorCurrentPosition(odometerYL) + bulkData.getMotorCurrentPosition(odometerYR)) / 2.0;
            double currentX = (double) bulkData.getMotorCurrentPosition(odometerX);
            double newWorldHeading = 0;

            double deltaAngle = newWorldHeading - worldPosition.heading; ////

            double deltaYodometer = currentY - Y_old;
            double deltaXodometer = currentX - X_old;

            double deltaPositionY = deltaYodometer;
            double deltaPositionX = deltaXodometer;

            if (deltaAngle != 0) {   //if deltaAngle = 0 radius of the arc is = Inf which causes model degeneracy

                double sinDeltaAngle = sin(deltaAngle); //will calculate sin and cos only once to optimize resources
                double cosDeltaAngle = MathUtil.cosFromSin(sinDeltaAngle, deltaAngle);

                double yOdometerArcRadius = deltaYodometer / deltaAngle; //calculate arc in which robot had travelled
                double xOdometerArcRadius = deltaXodometer / deltaAngle; //for each axis

                double yOdometerComponent = sinDeltaAngle * yOdometerArcRadius * cosDeltaAngle; //complex analytical geometry kicks in
                double xOdometerComponent = sinDeltaAngle * xOdometerArcRadius * sinDeltaAngle;

                deltaPositionY = yOdometerComponent - xOdometerComponent; //position change relative to the starting point (old)
                deltaPositionX = yOdometerComponent + xOdometerComponent;

            }

            double sinWorldAngle = sin(worldPosition.heading);
            double cosWorldAngle = cosFromSin(sinWorldAngle, worldPosition.heading);


            worldPosition.y += deltaPositionY * cosWorldAngle - deltaPositionX * sinWorldAngle; //rotate delta position according to the old robot heading
            worldPosition.x += deltaPositionY * sinWorldAngle + deltaPositionX * cosWorldAngle;
            worldPosition.heading = newWorldHeading;

            Y_old = currentY;
            X_old = currentX;

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }

    /**
     * Class initializer
     *
     * @param expansionHub ExpansionHubEx to which odometers are attached
     * @param odometerYL Left Y-axis encoder (looking from the top)
     * @param odometerYR Right Y-axis encoder (looking from the top)
     * @param odometerX X-axis encoder
     *
     */

    public Odometry(ExpansionHubEx expansionHub, ExpansionHubMotor odometerYL, ExpansionHubMotor odometerYR, ExpansionHubMotor odometerX)
    {
        this.expansionHub = expansionHub;
        this.odometerYL = odometerYL;
        this.odometerYR = odometerYR;
        this.odometerX = odometerX;
    }

    /**
     * Returns the robot's global coordinate
     *
     * @return robot position in (x,y,heading) form
     */
    public Pose2D getRobotCoordinate() {
        return worldPosition;
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