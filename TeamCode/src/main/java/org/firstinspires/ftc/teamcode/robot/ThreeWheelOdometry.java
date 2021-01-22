package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.superclasses.Odometry;
import org.openftc.revextensions2.ExpansionHubEx;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.math.MathUtil.angleAverage;
import static org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap;

public class ThreeWheelOdometry implements Odometry {
    private static final double odometryWheelDiameterCm = 4.8;
    private static final double odometryCountsPerCM = (1440) / (odometryWheelDiameterCm * PI);
    private static final double odometryCMPerCounts = (odometryWheelDiameterCm * PI) / 1440;
    private static final double odometerXcenterOffset = -36.8862986805 * odometryCountsPerCM * cos(toRadians(67.021303041)) / 2;
    private static final double yWheelPairRadiusCm = 18.2425;
    private static final double radiansPerEncoderDifference = 1.004604437*((odometryCMPerCounts) / (yWheelPairRadiusCm * 2));
    public static Pose2D worldPosition = new Pose2D();
    private static double angleOffset = 0;
    private static ExpansionHubEx expansionHub;
    private static BNO055IMU imu1;
    private static BNO055IMU imu2;
    public static DcMotorEx odometerYL = null;
    public static DcMotorEx odometerYR = null;
    public static DcMotorEx odometerX = null;
    private static LinearOpMode opMode = null;
  //  public RevBulkData bulkData;
    private float IMUoffset1 = 0;
    private float IMUoffset2 = 0;
    private double encoderHeadingCovariance = 0;
    private int YL_old = 0;
    private int YR_old = 0;
    private int X_old = 0;
    private final Vector2D YWheelPairCenterOffset = new Vector2D(0, 6.40008);
    private final ElapsedTime IMUAccessTimer = new ElapsedTime();


    /**
     * Class initializer
     */

    public ThreeWheelOdometry() {
    }

    private double calculateHeading() {
        if(IMUAccessTimer.seconds()>2)
        {
            double angleDivergence = angleWrap (getEncoderHeading(odometerYL.getCurrentPosition(),odometerYR.getTargetPosition())-angleAverage(getIMUheading_1(),getIMUheading_2()));
            if(abs(angleDivergence)>0.05) encoderHeadingCovariance = angleDivergence*(2.0/3.0);
            IMUAccessTimer.reset();
        }
        return angleWrap(getEncoderHeading(odometerYL.getCurrentPosition(), odometerYR.getCurrentPosition()) - angleOffset - encoderHeadingCovariance);
    }

    private double getEncoderHeading(double L, double R) {
        return (double) (L - R) * radiansPerEncoderDifference;
    }

    private void initIMU() {
        imu1 = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.mode = BNO055IMU.SensorMode.IMU;
        parameters1.calibrationDataFile = "BNO055IMUCalibration_1.json";
        imu1.initialize(parameters1);
        IMUoffset1 = -imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
        imu2 = opMode.hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.mode = BNO055IMU.SensorMode.NDOF;
        parameters2.calibrationDataFile = "BNO055IMUCalibration_2.json";
        imu2.initialize(parameters2);
        IMUoffset2 = -imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
        encoderHeadingCovariance = 0;
        IMUAccessTimer.reset();
    }

    private double getIMUheading_1() {
            return angleWrap(-imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle - IMUoffset1);
    }
    private double getIMUheading_2() {
        return angleWrap(-imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle - IMUoffset2);
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

        worldPosition = worldPosition.add(
                new Pose2D(deltaPosition.rotatedCW(worldPosition.heading),
                        deltaWorldHeading));

        YL_old = odometerYL.getCurrentPosition();
        YR_old = odometerYR.getCurrentPosition();
        X_old = odometerX.getCurrentPosition();
    }

    public void setOpMode(LinearOpMode opMode) {
        ThreeWheelOdometry.opMode = opMode;
    }

    public void initialize() {
        initIMU();

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

    public void setRobotCoordinates(Pose2D coordinates) {
        angleOffset = (double) angleWrap(calculateHeading() + angleOffset - coordinates.heading);
        this.calculatePosition(new Pose2D(new Vector2D(coordinates.x * odometryCountsPerCM, coordinates.y * odometryCountsPerCM)
                .add(YWheelPairCenterOffset.scale(odometryCountsPerCM).rotatedCW(worldPosition.heading)), coordinates.heading));
    }

    public Vector3D getRobotVelocity() {
        double angularVelocity = getEncoderHeading(odometerYL.getVelocity(), odometerYR.getVelocity());
        return new Vector3D(new Vector2D(
                ((double) odometerX.getVelocity() - angularVelocity * odometerXcenterOffset) * odometryCMPerCounts,
                (double) (odometerYL.getVelocity() + odometerYR.getVelocity()) * odometryCMPerCounts / 2)
                .rotatedCW(worldPosition.heading),
                angularVelocity);
    }

}