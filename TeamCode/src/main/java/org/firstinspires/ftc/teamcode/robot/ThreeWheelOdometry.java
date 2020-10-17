package org.firstinspires.ftc.teamcode.robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.math.MathUtil;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import javax.crypto.spec.GCMParameterSpec;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.math.MathUtil.cosFromSin;

public class ThreeWheelOdometry implements Runnable
{
    ExpansionHubMotor odometerYL, odometerYR, odometerX;
    ExpansionHubEx expansionHub;
    public RevBulkData bulkData;
    public Pose2D worldPosition = new Pose2D();

    ElapsedTime uptime = new ElapsedTime();

    private boolean doStop = false;

    public synchronized void doStop() {
        this.doStop = true;
    }

    private synchronized boolean keepRunning() {
        return !this.doStop;
    }

    public static final double odometryWheelDiameterCm = 4.8;
    public static final double odometryCountsPerCM = (1440) / (odometryWheelDiameterCm * PI);
    public static final double odometryCMPerCounts = (odometryWheelDiameterCm * PI) / 1440;
    public static final float odometerYcenterOffsetCm = 13;
    public static final float odometerXcenterOffsetCm = 2;
    public static final double yWheelPairRadiusCm = 18.1275;
    private static final double radiansPerEncoderDifference = (odometryCMPerCounts)/(yWheelPairRadiusCm*2);

    public double calculateHeading(int L, int R) {
        return (double)(L-R)*radiansPerEncoderDifference;
    }

    @Override
    public void run() {
        doStop = false;
        uptime.reset();
        bulkData = expansionHub.getBulkInputData();
        int  YL_old = bulkData.getMotorCurrentPosition(odometerYL);
        int  YR_old = -bulkData.getMotorCurrentPosition(odometerYR);
        int  X_old  = bulkData.getMotorCurrentPosition(odometerX);
        while (keepRunning()) {


            bulkData = expansionHub.getBulkInputData();

            //Get Current Positions
            int deltaYL = bulkData.getMotorCurrentPosition(odometerYL) - YL_old;
            int deltaYR = -bulkData.getMotorCurrentPosition(odometerYR) - YR_old;
            int deltaX  = bulkData.getMotorCurrentPosition(odometerX)  -  X_old;
            double calculatedHeading = calculateHeading(bulkData.getMotorCurrentPosition(odometerYL),-bulkData.getMotorCurrentPosition(odometerYR));
            double deltaWorldHeading = calculatedHeading - worldPosition.heading;

            double deltaYcomponent = (double)(deltaYL+deltaYR)/2.0;
            double deltaXcomponent = deltaX + deltaWorldHeading*0.0;

            Vector2D deltaPosition = new Vector2D(deltaXcomponent, deltaYcomponent);

            if (deltaWorldHeading != 0) {   //if deltaAngle = 0 radius of the arc is = Inf which causes model degeneracy
                double arcAngle = deltaPosition.acot();
                double arcRadius = deltaPosition.radius()/deltaWorldHeading;

                deltaPosition = new Vector2D(arcRadius*(1-cos(arcAngle)),arcRadius*sin(arcAngle)).rotatedCW(arcAngle);

            }
            worldPosition = worldPosition.add(new Pose2D(deltaPosition.rotatedCW(worldPosition.heading), deltaWorldHeading));

            YL_old = bulkData.getMotorCurrentPosition(odometerYL);
            YR_old = -bulkData.getMotorCurrentPosition(odometerYR);
            X_old  = bulkData.getMotorCurrentPosition(odometerX);

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
        odometerYL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerYR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        Vector2D poseTranslation = new Vector2D(worldPosition.x*odometryCMPerCounts,worldPosition.y*odometryCMPerCounts
        ).add(new Vector2D(0,1).rotatedCW(worldPosition.heading));
        return new Pose2D(poseTranslation,worldPosition.heading);
    }

}