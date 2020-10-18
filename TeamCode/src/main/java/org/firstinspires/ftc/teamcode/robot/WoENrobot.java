package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2D;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.signum;

public class WoENrobot{

    private WoENrobot() {
    }

    public static class SingletonHolder {
        public static final WoENrobot HOLDER_INSTANCE = new WoENrobot();
    }

    public static WoENrobot getInstance() {
        return SingletonHolder.HOLDER_INSTANCE;
    }

    public LinearOpMode opMode = null;

    /* Global runtime variables. */
    public static ElapsedTime Runtime = new ElapsedTime();
    public static boolean robotIsInitialized = false;
    public static ThreeWheelOdometry odometry = new ThreeWheelOdometry();
    public static Drivetrain drivetrain = new Drivetrain();
    public static TFdetector tFdetector = new TFdetector();
    public static Thread tFdetectorThread = new Thread(tFdetector);
    public static WobbleManipulator wobbleManipulator = new WobbleManipulator();


    private double robotEstimatedHeading;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0;
    private Pose2D WorldPosition;
    private float integratedAngle = 0;


    public void delay(double delay_ms)
    {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
                while(timer.milliseconds()<delay_ms && opMode.opModeIsActive());
    }
    public void startRobot() {
        opMode.waitForStart();
        regulatorUpdater.start();
        Runtime.reset();
        opMode.telemetry.addData("Status", "Running");
        opMode.telemetry.update();
    }

    public void initRobot(LinearOpMode opMode) {
        if (!robotIsInitialized) {
            forceInitRobot(opMode);
            opMode.telemetry.addData("Status", "Initialization successful");
            opMode.telemetry.update();
        } else {
            this.opMode = opMode;
            if (regulatorUpdater.getState() != Thread.State.NEW)
            {
                regulatorUpdater = new Thread(updateRegulators);
            }
            stopAllMotors();
            opMode.telemetry.addData("Status", "Already initialized, ready");
            opMode.telemetry.update();
        }
    }

    private static final boolean defaultNames = true;

    public void forceInitRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        opMode.telemetry.addData("Status", "Initializing...");
        opMode.telemetry.update();

        if (tFdetectorThread.getState() != Thread.State.NEW)
        {
            tFdetector.doStop();
            tFdetectorThread = new Thread(tFdetector);
        }
        if (regulatorUpdater.getState() != Thread.State.NEW)
        {
            regulatorUpdater = new Thread(updateRegulators);
        }
        drivetrain.initialize();
        wobbleManipulator.initialize();
        odometry.initialize();

        stopAllMotors();

        robotIsInitialized = true;
        opMode.telemetry.addData("Status", "Force initialized");
        opMode.telemetry.update();
    }

    public void stopAllMotors()
    {
        drivetrain.stopMotors();
    }


    Runnable updateRegulators = () -> {
        while(opMode.opModeIsActive()) {
            odometry.update();
            drivetrain.update();
            wobbleManipulator.update();
        }
    };

    private Thread regulatorUpdater = new Thread(updateRegulators);

  /*  public void initializeGyroscopes() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imuLeft.initialize(parameters);
        imuRight.initialize(parameters);

        imuLeftAngleOffset = -imuLeft.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        imuRightAngleOffset = -imuRight.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    } */

}

