package org.firstinspires.ftc.teamcode.robot;

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
import org.openftc.revextensions2.ExpansionHubEx;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap;

public class ThreeWheelOdometry implements Odometry{
    private static final double odometryWheelDiameterCm = 4.8;
    private static final double odometryCountsPerCM = (1440) / (odometryWheelDiameterCm * PI);
    private static final double odometryCMPerCounts = (odometryWheelDiameterCm * PI) / 1440;
    private static final double odometerXcenterOffset = 36.8862986805 * odometryCountsPerCM * cos(toRadians(67.021303041)) / 2;
    private static final double yWheelPairRadiusCm = 18.70558;//1870425;//18.1275;
    private static final double radiansPerEncoderDifference = ((odometryCMPerCounts)*1.0214214782 / (yWheelPairRadiusCm * 2));
   // private static final int odometerYL = 0, odometerYR = 1, odometerX = 2;
    public static Pose2D worldPosition = new Pose2D();
    private static double angleOffset = 0;
    private static ExpansionHubEx expansionHub;
    private static BNO055IMU imu;
    public static DcMotorEx odometerYL = null;
    public static DcMotorEx odometerYR = null;
    public static DcMotorEx odometerX = null;
    private static LinearOpMode opMode = null;
  //  public RevBulkData bulkData;
    private float IMUoffset = 0;
    private double encoderHeadingCovariance = 0;
    private int YL_old = 0;
    private int YR_old = 0;
    private int X_old = 0;

    /**
     * Class initializer
     */

    public ThreeWheelOdometry() {
    }


    private ElapsedTime IMUAccessTimer = new ElapsedTime();
    private double calculateHeading(int L, int R) {
        if(false)//IMUAccessTimer.seconds()>2)
        {
            double angleDivergence = angleWrap (angleWrap((double) (L - R) * radiansPerEncoderDifference)-getIMUheading());
            if(abs(angleDivergence)>0.05) encoderHeadingCovariance = angleDivergence;
            IMUAccessTimer.reset();
        }
        return angleWrap((double) (L - R) * radiansPerEncoderDifference - angleOffset - encoderHeadingCovariance);
    }

    private double calculateIncrementalHeading(double L, double R) {
        return (double) (L - R) * radiansPerEncoderDifference;
    }

    private void initIMU() {
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);
        WoENrobot.delay(100);
        IMUoffset = 0;//(float) getIMUheading();
        encoderHeadingCovariance = 0;
        IMUAccessTimer.reset();
    }

    private double getIMUheading() {
       // float angle1 = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
     //   float angle2 = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
            return angleWrap(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle - IMUoffset);
    }


    public void update() {
        calculatePosition(worldPosition);
    }

    public synchronized void calculatePosition(Pose2D initialPose) {

        worldPosition = initialPose;

     //   bulkData = expansionHub.getBulkInputData();

        double deltaWorldHeading = angleWrap(calculateHeading(odometerYL.getCurrentPosition(), -odometerYR.getCurrentPosition()) - worldPosition.heading);

        Vector2D deltaPosition = new Vector2D(
                (double) (-odometerX.getCurrentPosition() - X_old) - deltaWorldHeading * odometerXcenterOffset,
                (double) ((odometerYL.getCurrentPosition() - YL_old) + (-odometerYR.getCurrentPosition() - YR_old)) / 2);

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
        YR_old = -odometerYR.getCurrentPosition();
        X_old = -odometerX.getCurrentPosition();
    }

    public void setOpMode(LinearOpMode opMode) {
        ThreeWheelOdometry.opMode = opMode;
    }

    public void initialize() {
        initIMU();
        //expansionHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        //bulkData = expansionHub.getBulkInputData();
        odometerYL = opMode.hardwareMap.get(DcMotorEx.class, "odometerYL");
        odometerYR = opMode.hardwareMap.get(DcMotorEx.class, "odometerYR");
        odometerX = opMode.hardwareMap.get(DcMotorEx.class, "odometerXConveyor");

        YL_old = odometerYL.getCurrentPosition();
        YR_old = -odometerYR.getCurrentPosition();
        X_old = -odometerX.getCurrentPosition();
    }

    /**
     * Returns the robot's global coordinates
     *
     * @return robot position in (x,y,heading) form
     */
    public Pose2D getRobotCoordinates() {
        Vector2D poseTranslation = new Vector2D(worldPosition.x * odometryCMPerCounts, worldPosition.y * odometryCMPerCounts
        );//.add(new Vector2D(0, 1).rotatedCW(worldPosition.heading));
        return new Pose2D(poseTranslation, worldPosition.heading);
    }

    public void setRobotCoordinates(Pose2D coordinates) {
        angleOffset = (double) angleWrap(calculateHeading(odometerYL.getCurrentPosition(), -odometerYR.getCurrentPosition()) + angleOffset - coordinates.heading);
        this.calculatePosition(new Pose2D(coordinates.x * odometryCountsPerCM, coordinates.y * odometryCountsPerCM, coordinates.heading));
    }

    public Vector3D getRobotVelocity() {
        double angularVelocity = calculateIncrementalHeading(odometerYL.getVelocity(), -odometerYR.getVelocity());
        return new Vector3D(new Vector2D(
                ((double) -odometerX.getVelocity() - angularVelocity * odometerXcenterOffset) * odometryCMPerCounts,
                (double) (odometerYL.getVelocity() - odometerYR.getVelocity()) * odometryCMPerCounts / 2)
                .rotatedCW(worldPosition.heading),
                angularVelocity);
    }

}