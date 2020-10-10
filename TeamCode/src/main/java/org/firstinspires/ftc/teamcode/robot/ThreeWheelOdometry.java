package org.firstinspires.ftc.teamcode.robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.math.MathUtil;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.math.MathUtil.cosFromSin;

public class ThreeWheelOdometry implements Runnable
{
    ExpansionHubMotor odometerYL, odometerYR, odometerX;
    ExpansionHubEx expansionHub;
    RevBulkData bulkData;
    Pose2D worldPosition = new Pose2D();

    ElapsedTime uptime = new ElapsedTime();

    private boolean doStop = false;

    public synchronized void doStop() {
        this.doStop = true;
    }

    private synchronized boolean keepRunning() {
        return !this.doStop;
    }

    @Override
    public void run() {
        doStop = false;
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

            double deltaWorldHeading = newWorldHeading - worldPosition.heading; ////

            double deltaYodometer = currentY - Y_old;
            double deltaXodometer = currentX - X_old;

            Vector2D deltaPosition = new Vector2D(deltaXodometer, deltaYodometer);

            if (deltaWorldHeading != 0) {   //if deltaAngle = 0 radius of the arc is = Inf which causes model degeneracy

                double arcAngle = deltaPosition.acot();
                double arcRadius = arcAngle/deltaWorldHeading;

                deltaPosition = new Vector2D(arcRadius*(1-cos(arcAngle)),arcRadius*sin(arcAngle)).rotatedCW(arcAngle);

            }

           worldPosition = worldPosition.add(new Pose2D(deltaPosition.rotatedCW(worldPosition.heading), deltaWorldHeading));

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
     */

    public ThreeWheelOdometry()
    {
    }

    public void initialize() {
        assignNames();
    }

    private void assignNames() {
        expansionHub = WoENrobot.getInstance().opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        odometerYL = (ExpansionHubMotor) WoENrobot.getInstance().opMode.hardwareMap.dcMotor.get("odometerYL");
        odometerYR = (ExpansionHubMotor) WoENrobot.getInstance().opMode.hardwareMap.dcMotor.get("odometerYL");
        odometerX = (ExpansionHubMotor) WoENrobot.getInstance().opMode.hardwareMap.dcMotor.get("odometerX");
    }

    /**
     * Returns the robot's global coordinates
     *
     * @return robot position in (x,y,heading) form
     */
    public Pose2D getRobotCoordinates() {
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