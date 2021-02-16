package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.superclasses.Odometry;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.jetbrains.annotations.NotNull;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.math.MathUtil.angleAverage;
import static org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap;

public class ThreeWheelOdometry extends RobotModule implements Odometry {
    private double odometryWheelDiameterCm = OdometryConfig.forwardMultiplier*4.8;
    private double odometryCountsPerCM = (1440) / (odometryWheelDiameterCm * PI);
    private double odometryCMPerCounts = (odometryWheelDiameterCm * PI) / 1440;
    private double odometerXcenterOffset = -21.7562349 * odometryCountsPerCM * cos(toRadians(51.293002));
    private static final double yWheelPairRadiusCm = 18.2425;
    private double radiansPerEncoderDifference = OdometryConfig.headingMultiplier * ((odometryCMPerCounts) / (yWheelPairRadiusCm * 2));
    private static Pose2D worldPosition = new Pose2D();
    private static double angleOffset = 0;
    private static BNO055IMU imu1;
    private static BNO055IMU imu2;
    public static DcMotorEx odometerYL = null;
    public static DcMotorEx odometerYR = null;
    public static DcMotorEx odometerX = null;
    private float IMUoffset1 = 0;
    private float IMUoffset2 = 0;
    private double encoderHeadingCovariance = 0;
    private int YL_old = 0;
    private int YR_old = 0;
    private int X_old = 0;
    private final Vector2D YWheelPairCenterOffset = new Vector2D(0, 6.40008);
    private final ElapsedTime IMUAccessTimer = new ElapsedTime();

    @Config
    static class OdometryConfig {
        public static double forwardMultiplier = 1.00;
        public static double headingMultiplier = 1.003851129713462;
        public static boolean doUseIMU = false;
    }

    private boolean doUseIMU_local = OdometryConfig.doUseIMU;


    /**
     * Class initializer
     */

    public ThreeWheelOdometry() {
    }

    private double calculateHeading() {
        if (doUseIMU_local && IMUAccessTimer.seconds()>1)
        {
            double angleDivergence = angleWrap(getEncoderHeading() - angleAverage(getIMUheading_1(), getIMUheading_2()));
            encoderHeadingCovariance = angleDivergence * (1.0 / 2.0);
            IMUAccessTimer.reset();
        }
        return angleWrap(getEncoderHeading() - angleOffset - encoderHeadingCovariance);
    }

    public double getEncoderHeading() {
        return getEncoderHeading(odometerYL.getCurrentPosition(), odometerYR.getCurrentPosition());
    }

    private double getEncoderHeading(double L, double R) {
        return (L - R) * radiansPerEncoderDifference;
    }

    private void initIMU() {
        imu1 = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.accelRange = BNO055IMU.AccelRange.G2;
        parameters1.gyroRange = BNO055IMU.GyroRange.DPS500;
        parameters1.mode = BNO055IMU.SensorMode.IMU;
        parameters1.calibrationDataFile = "BNO055IMUCalibration_1.json";
        imu1.initialize(parameters1);
        imu2 = opMode.hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.accelRange = BNO055IMU.AccelRange.G2;
        parameters2.gyroRange = BNO055IMU.GyroRange.DPS500;
        parameters2.mode = BNO055IMU.SensorMode.IMU;
        parameters2.calibrationDataFile = "BNO055IMUCalibration_2.json";
        imu2.initialize(parameters2);
        encoderHeadingCovariance = 0;
        IMUAccessTimer.reset();
    }

    public double getIMUheading_1() {
        if(doUseIMU_local)
        return angleWrap(-imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle - IMUoffset1);
        return -0;
    }

    public double getIMUheading_2() {
        if(doUseIMU_local)
        return angleWrap(-imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle - IMUoffset2);
        return -0;
    }


    public void start() {
        if(doUseIMU_local) {
            IMUoffset1 = -imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
            IMUoffset2 = -imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
        }
    }

    public void update() {
        calculatePosition(worldPosition);
    }

    public synchronized void calculatePosition(Pose2D initialPose) {

        worldPosition = initialPose;

        double deltaWorldHeading = angleWrap(calculateHeading() - worldPosition.heading);

        Vector2D deltaPosition = new Vector2D(
                (double) (odometerX.getCurrentPosition() - X_old) - deltaWorldHeading * odometerXcenterOffset,
                (double) ((odometerYL.getCurrentPosition() - YL_old) + (odometerYR.getCurrentPosition() - YR_old)) / 2);

        if (deltaWorldHeading != 0) {   //if deltaAngle = 0 radius of the arc is = Inf which causes model degeneracy
            double arcAngle = deltaWorldHeading * 2;
            double arcRadius = deltaPosition.radius() / arcAngle;

            deltaPosition = new Vector2D(
                    arcRadius * (1 - cos(arcAngle)),
                    arcRadius * sin(arcAngle))
                    .rotatedCW(deltaPosition.acot());
        }

        worldPosition = worldPosition.plus(
                new Pose2D(deltaPosition.rotatedCW(worldPosition.heading),
                        deltaWorldHeading));

        YL_old = odometerYL.getCurrentPosition();
        YR_old = odometerYR.getCurrentPosition();
        X_old = odometerX.getCurrentPosition();
    }

    public void initialize() {
        doUseIMU_local = OdometryConfig.doUseIMU;
        if(doUseIMU_local)
        initIMU();

        radiansPerEncoderDifference = OdometryConfig.headingMultiplier * ((odometryCMPerCounts) / (yWheelPairRadiusCm * 2));
        odometryWheelDiameterCm = OdometryConfig.forwardMultiplier*4.8;
        odometryCountsPerCM = (1440) / (odometryWheelDiameterCm * PI);
        odometryCMPerCounts = (odometryWheelDiameterCm * PI) / 1440;
        odometerXcenterOffset = -21.7562349 * odometryCountsPerCM * cos(toRadians(51.293002));

        odometerYL = opMode.hardwareMap.get(DcMotorEx.class, "odometerYL");
        odometerYR = opMode.hardwareMap.get(DcMotorEx.class, "odometerYR");
        odometerX = opMode.hardwareMap.get(DcMotorEx.class, "odometerXConveyor");

        odometerYL.setDirection(DcMotorEx.Direction.FORWARD);
        odometerYR.setDirection(DcMotorEx.Direction.REVERSE);
        odometerX.setDirection(DcMotorEx.Direction.REVERSE);
        odometerYL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometerYR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometerX.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometerYL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        odometerYR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        odometerX.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        YL_old = odometerYL.getCurrentPosition();
        YR_old = odometerYR.getCurrentPosition();
        X_old = odometerX.getCurrentPosition();
    }

    /**
     * Returns the robot's global coordinates
     *
     * @return robot position in (x,y,heading) form
     */
    public Pose2D getRobotCoordinates() {
        Vector2D poseTranslation = new Vector2D(worldPosition.x * odometryCMPerCounts, worldPosition.y * odometryCMPerCounts
        ).minus(YWheelPairCenterOffset.rotatedCW(worldPosition.heading));
        return new Pose2D(poseTranslation, worldPosition.heading);
    }

    public void setRobotCoordinates(@NotNull Pose2D coordinates) {
        YL_old = odometerYL.getCurrentPosition();
        YR_old = odometerYR.getCurrentPosition();
        X_old = odometerX.getCurrentPosition();
        angleOffset = angleWrap(calculateHeading() + angleOffset - coordinates.heading);
        this.calculatePosition(new Pose2D(new Vector2D(coordinates.x * odometryCountsPerCM, coordinates.y * odometryCountsPerCM)
                .plus(YWheelPairCenterOffset.times(odometryCountsPerCM).rotatedCW(worldPosition.heading)), coordinates.heading));
    }

    public Vector3D getRobotVelocity() {
        double angularVelocity = getEncoderHeading(odometerYL.getVelocity(), odometerYR.getVelocity());
        return new Vector3D(new Vector2D(
                (odometerX.getVelocity() - angularVelocity * odometerXcenterOffset) * odometryCMPerCounts,
                (odometerYL.getVelocity() + odometerYR.getVelocity()) * odometryCMPerCounts / 2)
                .rotatedCW(worldPosition.heading),
                angularVelocity);
    }

}