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
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap;

@Deprecated
public class TwoWheelOdometry extends RobotModule implements Odometry {
    private static final double odometryWheelDiameterCm = 4.8;
    private static final double odometryCountsPerCM = (1440) / (odometryWheelDiameterCm * PI);
    private static final double odometryCMPerCounts = (odometryWheelDiameterCm * PI) / 1440;
    private static final double odometerYcenterOffset = 38.3633669516 * odometryCountsPerCM * cos(toRadians(19.490773014)) / 2;
    private static final double odometerXcenterOffset = 36.8862986805 * odometryCountsPerCM * cos(toRadians(67.021303041)) / 2;
    private static final double yWheelPairRadiusCm = 18.425;//18.1275;
    private static final double radiansPerEncoderDifference = (odometryCMPerCounts) / (yWheelPairRadiusCm * 2);
    private static final int odometerY = 1, odometerX = 2;
    public static Pose2D worldPosition = new Pose2D();
    private static float IMUoffset = 0;
    private static BNO055IMU imu;
    private static ExpansionHubEx expansionHub;
    public RevBulkData bulkData;
    ElapsedTime looptime = new ElapsedTime();
    double angle_last = 0;
    private int Y_old = 0;
    private int X_old = 0;

    /**
     * Class initializer
     */

    public TwoWheelOdometry() {
    }

    private double calculateHeading(int L, int R) {
        return angleWrap((double) (L - R) * radiansPerEncoderDifference - IMUoffset);
    }

    private void initIMU() {
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());
        IMUoffset = (float) getIMUheading();
    }

    private double getIMUheading() {
        if (looptime.milliseconds() > 50) {
            looptime.reset();
            return angle_last = angleWrap(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle - IMUoffset);
        } else return angle_last;
    }

    public void update() {
        calculatePosition(worldPosition);
    }

    public synchronized void calculatePosition(Pose2D initialPose) {

        worldPosition = initialPose;

        bulkData = expansionHub.getBulkInputData();

        double deltaWorldHeading = angleWrap(getIMUheading() - worldPosition.heading);

        Vector2D deltaPosition = new Vector2D((double) (bulkData.getMotorCurrentPosition(odometerX) - X_old) - deltaWorldHeading * odometerXcenterOffset,
                (double) (-bulkData.getMotorCurrentPosition(odometerY) - Y_old) - deltaWorldHeading * odometerYcenterOffset);

        worldPosition = worldPosition.plus(new Pose2D(deltaPosition.rotatedCW(worldPosition.heading + deltaWorldHeading / 2), deltaWorldHeading));
        Y_old = -bulkData.getMotorCurrentPosition(odometerY);
        X_old = bulkData.getMotorCurrentPosition(odometerX);

    }

    public void initialize() {
        assignNames();
        initIMU();
        WoENrobot.delay(500);
        bulkData = expansionHub.getBulkInputData();
        Y_old = -bulkData.getMotorCurrentPosition(odometerY);
        X_old = bulkData.getMotorCurrentPosition(odometerX);
    }

    private void assignNames() {
        expansionHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
    }

    /**
     * Returns the robot's global coordinates
     *
     * @return robot position in (x,y,heading) form
     */

    @Override
    public Pose2D getRobotCoordinates() {
        Vector2D poseTranslation = new Vector2D(worldPosition.x * odometryCMPerCounts, worldPosition.y * odometryCMPerCounts
        );//.add(new Vector2D(0, 1).rotatedCW(worldPosition.heading));
        return new Pose2D(poseTranslation, worldPosition.heading);
    }

    public void setRobotCoordinates(Pose2D coordinates) {
        IMUoffset = (float) angleWrap(calculateHeading(bulkData.getMotorCurrentPosition(odometerY), -bulkData.getMotorCurrentPosition(odometerY)) + IMUoffset - coordinates.heading);
        this.calculatePosition(new Pose2D(coordinates.x * odometryCountsPerCM, coordinates.y * odometryCountsPerCM, coordinates.heading));
    }

    public Vector3D getRobotVelocity() {
        double angularVelocity = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
        return new Vector3D(new Vector2D(
                ((double) bulkData.getMotorCurrentPosition(odometerX) - angularVelocity * odometerXcenterOffset) * odometryCMPerCounts,
                ((double) -bulkData.getMotorVelocity(odometerY) - angularVelocity * odometerXcenterOffset) * odometryCMPerCounts)
                .rotatedCW(worldPosition.heading),
                angularVelocity);
    }

}