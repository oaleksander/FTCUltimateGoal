package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.superclasses.Odometry;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap;

public class ThreeWheelOdometry implements Odometry, RobotModule {
    private static final double odometryWheelDiameterCm = 4.8;
    private static final double odometryCountsPerCM = (1440) / (odometryWheelDiameterCm * PI);
    private static final double odometryCMPerCounts = (odometryWheelDiameterCm * PI) / 1440;
    private static final double odometerXcenterOffset = 36.8862986805 * odometryCountsPerCM * cos(toRadians(67.021303041)) / 2;
    private static final double yWheelPairRadiusCm = 18.1275;
    private static final double radiansPerEncoderDifference = (odometryCMPerCounts) / (yWheelPairRadiusCm * 2);
    private static final int odometerYL = 0, odometerYR = 1, odometerX = 2;
    public static Pose2D worldPosition = new Pose2D();
    private static double angleOffset = 0;
    private static ExpansionHubEx expansionHub;
    private static LinearOpMode opMode = null;
    public RevBulkData bulkData;
    private int YL_old = 0;
    private int YR_old = 0;
    private int X_old = 0;

    /**
     * Class initializer
     */

    public ThreeWheelOdometry() {
    }

    private double calculateHeading(int L, int R) {
        return angleWrap((double) (L - R) * radiansPerEncoderDifference- angleOffset);
    }

    private double calculateIncrementalHeading(int L, int R)
    {
        return (double) (L - R) * radiansPerEncoderDifference;
    }

    public void update() {
            calculatePosition(worldPosition);
    }

    public synchronized void calculatePosition(Pose2D initialPose) {

        worldPosition = initialPose;

        bulkData = expansionHub.getBulkInputData();

        double deltaWorldHeading = angleWrap(calculateHeading(bulkData.getMotorCurrentPosition(odometerYL),-bulkData.getMotorCurrentPosition(odometerYR)) - worldPosition.heading);

        Vector2D deltaPosition = new Vector2D(
                (double) (bulkData.getMotorCurrentPosition(odometerX) - X_old) - deltaWorldHeading * odometerXcenterOffset,
                (double)( (bulkData.getMotorCurrentPosition(odometerYL) - YL_old)+(-bulkData.getMotorCurrentPosition(odometerYR) - YR_old))/2);

        if (deltaWorldHeading != 0) {   //if deltaAngle = 0 radius of the arc is = Inf which causes model degeneracy
            double arcAngle = deltaWorldHeading * 2;
            double arcRadius = deltaPosition.radius() / arcAngle;

            deltaPosition = new Vector2D(
                    arcRadius * (1 - cos(arcAngle)),
                    arcRadius * sin(arcAngle))
                    .rotatedCW(deltaPosition.acot());

        }

        worldPosition = worldPosition.add(
                new Pose2D(deltaPosition.rotatedCW(worldPosition.heading),
                        deltaWorldHeading));

        YL_old = bulkData.getMotorCurrentPosition(odometerYL);
        YR_old = -bulkData.getMotorCurrentPosition(odometerYR);
        X_old = bulkData.getMotorCurrentPosition(odometerX);
    }

    public void setOpMode(LinearOpMode opMode) {
        ThreeWheelOdometry.opMode = opMode;
    }

    public void initialize() {
        expansionHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        bulkData = expansionHub.getBulkInputData();
        YL_old = bulkData.getMotorCurrentPosition(odometerYL);
        YR_old = -bulkData.getMotorCurrentPosition(odometerYR);
        X_old = bulkData.getMotorCurrentPosition(odometerX);
    }

    public Pose2D getRobotCoordinates() {
        Vector2D poseTranslation = new Vector2D(worldPosition.x * odometryCMPerCounts, worldPosition.y * odometryCMPerCounts
        );//.add(new Vector2D(0, 1).rotatedCW(worldPosition.heading));
        return new Pose2D(poseTranslation, worldPosition.heading);
    }
    public Vector3D getRobotVelocity()
    {
        double angularVelocity = calculateIncrementalHeading(bulkData.getMotorCurrentPosition(odometerYL),-bulkData.getMotorCurrentPosition(odometerYR));
        return new Vector3D(
                ((double)bulkData.getMotorCurrentPosition(odometerX)-angularVelocity*odometerXcenterOffset)* odometryCMPerCounts,
                (double)(bulkData.getMotorVelocity(odometerYL)-bulkData.getMotorVelocity(odometerYR)) * odometryCMPerCounts/2,
                angularVelocity);
    }

    /**
     * Returns the robot's global coordinates
     *
     * @return robot position in (x,y,heading) form
     */
    public void setRobotCoordinates(Pose2D coordinates) {
        angleOffset = (double) angleWrap(calculateHeading(bulkData.getMotorCurrentPosition(odometerYL),-bulkData.getMotorCurrentPosition(odometerYR)) + angleOffset - coordinates.heading);
        this.calculatePosition(new Pose2D(coordinates.x * odometryCountsPerCM, coordinates.y * odometryCountsPerCM, coordinates.heading));
    }

}