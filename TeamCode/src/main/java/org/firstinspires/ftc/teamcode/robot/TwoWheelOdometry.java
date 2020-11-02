package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.superclasses.Odometry;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap;

public class TwoWheelOdometry implements Odometry {
    private static final double odometryWheelDiameterCm = 4.8;
    private static final double odometryCountsPerCM = (1440) / (odometryWheelDiameterCm * PI);
    private static final double odometryCMPerCounts = (odometryWheelDiameterCm * PI) / 1440;
    private static final double odometerYcenterOffset = 38.3633669516 * odometryCountsPerCM * cos(toRadians(19.490773014)) / 2;
    private static final double odometerXcenterOffset = 36.8862986805 * odometryCountsPerCM * cos(toRadians(67.021303041)) / 2;
    private static final double yWheelPairRadiusCm = 18.1275;
    private static final double radiansPerEncoderDifference = (odometryCMPerCounts) / (yWheelPairRadiusCm * 2);
    private static float IMUoffset = 0;
    private static int YL_old = 0;
    private static int YR_old = 0;
    private static int X_old = 0;
    public static RevBulkData bulkData;
    public static Pose2D worldPosition = new Pose2D();
    private static BNO055IMU imu;
    private static ExpansionHubMotor odometerYL, odometerX;
    private static ExpansionHubEx expansionHub;
    ElapsedTime looptime = new ElapsedTime();
    private static LinearOpMode opMode = null;
    /**
     * Class initializer
     */

    public TwoWheelOdometry() {
    }

    private double calculateHeading(int L, int R) {
        return (double) (L - R) * radiansPerEncoderDifference;
    }

    private void initIMU() {
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());
        IMUoffset = (float) getIMUheading();
    }

    private double getIMUheading() {
        return angleWrap(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle - IMUoffset);
    }

    public void update() {
        update(worldPosition);
    }

    public synchronized void update(Pose2D initialPose) {

        worldPosition = initialPose;

        bulkData = expansionHub.getBulkInputData();

        //Get Current Positions
        int deltaYL = bulkData.getMotorCurrentPosition(odometerYL) - YL_old;
        //int deltaYR = -bulkData.getMotorCurrentPosition(odometerYR) - YR_old;
        int deltaX = bulkData.getMotorCurrentPosition(odometerX) - X_old;
        double calculatedHeading = getIMUheading();
        //double calculatedHeading = calculateHeading(bulkData.getMotorCurrentPosition(odometerYL),-bulkData.getMotorCurrentPosition(odometerYR));
        double deltaWorldHeading = angleWrap(calculatedHeading - worldPosition.heading);

        //double deltaYcomponent = (double)(deltaYL+deltaYR)/2.0;
        double deltaYcomponent = deltaYL - deltaWorldHeading * odometerYcenterOffset;
        double deltaXcomponent = deltaX - deltaWorldHeading * odometerXcenterOffset;

        Vector2D deltaPosition = new Vector2D(deltaXcomponent, deltaYcomponent);

            /* if (deltaWorldHeading != 0 && false) {   //if deltaAngle = 0 radius of the arc is = Inf which causes model degeneracy
                double arcAngle = deltaPosition.acot();
                double arcRadius = deltaPosition.radius()/deltaWorldHeading;

                deltaPosition = new Vector2D(arcRadius*(1-cos(arcAngle)),arcRadius*sin(arcAngle)).rotatedCW(arcAngle);

            } */
        worldPosition = worldPosition.add(new Pose2D(deltaPosition.rotatedCW(worldPosition.heading + deltaWorldHeading / 2), deltaWorldHeading));

        YL_old = bulkData.getMotorCurrentPosition(odometerYL);
        //YR_old = -bulkData.getMotorCurrentPosition(odometerYR);
        X_old = bulkData.getMotorCurrentPosition(odometerX);

    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initialize() {
        assignNames();
        initIMU();
        WoENrobot.delay(500);
        odometerYL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //odometerYR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bulkData = expansionHub.getBulkInputData();
        YL_old = bulkData.getMotorCurrentPosition(odometerYL);
        //YR_old = -bulkData.getMotorCurrentPosition(odometerYR);
        X_old = bulkData.getMotorCurrentPosition(odometerX);
    }

    private void assignNames() {
        expansionHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        odometerYL = (ExpansionHubMotor) opMode.hardwareMap.dcMotor.get("odometerYL");
        //odometerYR = (ExpansionHubMotor) opMode.hardwareMap.dcMotor.get("odometerYL");
        odometerX = (ExpansionHubMotor) opMode.hardwareMap.dcMotor.get("odometerX");
    }

    public Pose2D getRobotCoordinates() {
        Vector2D poseTranslation = new Vector2D(worldPosition.x * odometryCMPerCounts, worldPosition.y * odometryCMPerCounts
        ).add(new Vector2D(0, 1).rotatedCW(worldPosition.heading));
        return new Pose2D(poseTranslation, worldPosition.heading);
    }

    /**
     * Returns the robot's global coordinates
     *
     * @return robot position in (x,y,heading) form
     */
    public void setRobotCoordinates(Pose2D coordinates) {
        IMUoffset = (float) angleWrap(getIMUheading() + IMUoffset - coordinates.heading);
        this.update(new Pose2D(coordinates.x * odometryCountsPerCM, coordinates.y * odometryCountsPerCM, coordinates.heading));
    }

}