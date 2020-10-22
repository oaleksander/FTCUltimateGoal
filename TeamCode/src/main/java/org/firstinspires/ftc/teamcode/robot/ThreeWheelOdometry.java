package org.firstinspires.ftc.teamcode.robot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
import static java.lang.Math.toRadians;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap;
import static org.firstinspires.ftc.teamcode.math.MathUtil.cosFromSin;

public class ThreeWheelOdometry
{
    BNO055IMU imu;
    ExpansionHubMotor odometerYL, odometerYR, odometerX;
    ExpansionHubEx expansionHub;
    public RevBulkData bulkData;
    public Pose2D worldPosition = new Pose2D();

    ElapsedTime looptime = new ElapsedTime();

    public static final double odometryWheelDiameterCm = 4.8;
    public static final double odometryCountsPerCM = (1440) / (odometryWheelDiameterCm * PI);
    public static final double odometryCMPerCounts = (odometryWheelDiameterCm * PI) / 1440;
    public static final double odometerYcenterOffset = 38.3633669516*odometryCountsPerCM*cos(toRadians(19.490773014))/2;
    public static final double odometerXcenterOffset = 36.8862986805*odometryCountsPerCM*cos(toRadians(67.021303041))/2;
    public static final double yWheelPairRadiusCm = 18.1275;
    private static final double radiansPerEncoderDifference = (odometryCMPerCounts)/(yWheelPairRadiusCm*2);

    public double calculateHeading(int L, int R) {
        return (double)(L-R)*radiansPerEncoderDifference;
    }

    static float IMUoffset = 0;
    public void initIMU()
    {
        imu = WoENrobot.getInstance().opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());
        IMUoffset = (float)getIMUheading();
    }

    public double getIMUheading(){
        return angleWrap(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle - IMUoffset);
    }

    int  YL_old = 0;
    int  YR_old = 0;
    int  X_old = 0;

   // @Override
    public void update()
    {
        update(worldPosition);
    }
    public synchronized void update(Pose2D initialPose) {

            worldPosition = initialPose;

            bulkData = expansionHub.getBulkInputData();

            //Get Current Positions
            int deltaYL = bulkData.getMotorCurrentPosition(odometerYL) - YL_old;
            int deltaYR = -bulkData.getMotorCurrentPosition(odometerYR) - YR_old;
            int deltaX  = bulkData.getMotorCurrentPosition(odometerX)  -  X_old;
            double calculatedHeading = getIMUheading();
            //double calculatedHeading = calculateHeading(bulkData.getMotorCurrentPosition(odometerYL),-bulkData.getMotorCurrentPosition(odometerYR));
            double deltaWorldHeading = angleWrap(calculatedHeading - worldPosition.heading);

            //double deltaYcomponent = (double)(deltaYL+deltaYR)/2.0;
            double deltaYcomponent = deltaYL - deltaWorldHeading*odometerYcenterOffset;
            double deltaXcomponent = deltaX - deltaWorldHeading*odometerXcenterOffset;

            Vector2D deltaPosition = new Vector2D(deltaXcomponent, deltaYcomponent);

            /* if (deltaWorldHeading != 0 && false) {   //if deltaAngle = 0 radius of the arc is = Inf which causes model degeneracy
                double arcAngle = deltaPosition.acot();
                double arcRadius = deltaPosition.radius()/deltaWorldHeading;

                deltaPosition = new Vector2D(arcRadius*(1-cos(arcAngle)),arcRadius*sin(arcAngle)).rotatedCW(arcAngle);

            } */
            worldPosition = worldPosition.add(new Pose2D(deltaPosition.rotatedCW(worldPosition.heading+deltaWorldHeading/2), deltaWorldHeading));

            YL_old = bulkData.getMotorCurrentPosition(odometerYL);
            YR_old = -bulkData.getMotorCurrentPosition(odometerYR);
            X_old  = bulkData.getMotorCurrentPosition(odometerX);

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
        initIMU();
        WoENrobot.getInstance().delay(500);
        odometerYL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerYR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bulkData = expansionHub.getBulkInputData();
        YL_old = bulkData.getMotorCurrentPosition(odometerYL);
        YR_old = -bulkData.getMotorCurrentPosition(odometerYR);
        X_old  = bulkData.getMotorCurrentPosition(odometerX);
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
    public void setRobotCoordinates(Pose2D coordinates)
    {
        IMUoffset = (float)angleWrap(getIMUheading()+IMUoffset-coordinates.heading);
        this.update(new Pose2D(coordinates.x*odometryCountsPerCM, coordinates.y*odometryCountsPerCM, coordinates.heading));
    }
    public Pose2D getRobotCoordinates() {
        Vector2D poseTranslation = new Vector2D(worldPosition.x*odometryCMPerCounts,worldPosition.y*odometryCMPerCounts
        ).add(new Vector2D(0,1).rotatedCW(worldPosition.heading));
        return new Pose2D(poseTranslation,worldPosition.heading);
    }

}