package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.Conveyor;
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain;
import org.firstinspires.ftc.teamcode.superclasses.Odometry;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.Arrays;
import java.util.List;

public class WoENrobot {

    public static WobbleManipulator wobbleManipulator = new ServoWobbleManipulator();
    public static OpenCVNodeWebcam openCVNode = new OpenCVNodeWebcam();
    public static Conveyor conveyor = new Conveyor2();
    public static rpm shooter = new rpm();
    public static TelemetryDebugging telemetryDebugging = new TelemetryDebugging();

    //public static FakeRobot fakeRobot = new FakeRobot();
    //public static Odometry odometry = fakeRobot;
    //public static Drivetrain drivetrain = fakeRobot;

    public static ThreeWheelOdometry odometry = new ThreeWheelOdometry();
    public static MecanumDrivetrain drivetrain = new MecanumDrivetrain();
    public static Movement movement = new Movement(odometry, drivetrain);
    public static LinearOpMode opMode = null;
    public static boolean robotIsInitialized = false;
    public static final ElapsedTime runTime = new ElapsedTime();
    protected static RobotModule[] activeRobotModules = {odometry, movement, drivetrain, shooter, wobbleManipulator, conveyor, telemetryDebugging}; //conveyor, odometry, shooter, wobbleManipulator, drivetrain
    static boolean spinCompleted = false;
    private static ExpansionHubEx expansionHub1 = null;
    private static ExpansionHubEx expansionHub2 = null;
    private static List<LynxModule> allHubs = null;
    static Runnable updateRegulators = () -> {
        setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        while (opMode.opModeIsActive() && !Thread.interrupted()) {
            clearBulkCaches();
            Arrays.stream(activeRobotModules).forEach(RobotModule::update);
            spinCompleted = true;
        }
    };
    private static Thread regulatorUpdater = new Thread(updateRegulators);

    public static LinearOpMode getOpMode() {
        return opMode;
    }

    public static void FullInitWithCV(LinearOpMode opMode) {
        openCVNode.initialize(opMode);
        forceInitRobot(opMode);
    }

    public static void spinOnce() {
        spinCompleted = false;
        while (!spinCompleted && opMode.opModeIsActive()) {
            Thread.yield();
        }
    }

    public static void delay(double delay_ms) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < delay_ms && opMode.opModeIsActive()) {
            Thread.yield();
        }
    }

    public static void startRobot() {
        setLedColors(255, 166, 0);
        opMode.waitForStart();
        regulatorUpdater.start();
        runTime.reset();
        setLedColors(0, 237, 255);
        opMode.telemetry.addData("Status", "Running");
        opMode.telemetry.update();
    }

    public static void initRobot(LinearOpMode OpMode) {
        if (!robotIsInitialized) {
            forceInitRobot(OpMode);
            opMode.telemetry.addData("Status", "Initialization successful");
            opMode.telemetry.update();
        } else {
            opMode = OpMode;
            Arrays.stream(activeRobotModules).forEach(RobotModule::start);
            if (regulatorUpdater.getState() != Thread.State.NEW) {
                regulatorUpdater.interrupt();
                regulatorUpdater = new Thread(updateRegulators);
            }
            opMode.telemetry.addData("Status", "Already initialized, ready");
            opMode.telemetry.update();
        }
    }

    public static void forceInitRobot(LinearOpMode OpMode) {
        opMode = OpMode;

        opMode.telemetry.addData("Status", "Initializing...");
        opMode.telemetry.update();


        allHubs = opMode.hardwareMap.getAll(LynxModule.class);
        expansionHub1 = opMode.hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        expansionHub2 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        Arrays.stream(activeRobotModules).forEach(robotModule -> robotModule.initialize(opMode));

        if (regulatorUpdater.getState() != Thread.State.NEW) {
            regulatorUpdater.interrupt();
            regulatorUpdater = new Thread(updateRegulators);
        }
        //stopAllMotors();

        robotIsInitialized = true;

        opMode.telemetry.addData("Status", "Force initialized");
        opMode.telemetry.update();

    }

    public static void simpleInit(LinearOpMode OpMode) {
        initRobot(opMode);
        startRobot();
    }

    public static void setLedColors(int r, int g, int b) {
        expansionHub1.setLedColor(r, g, b);
        expansionHub2.setLedColor(r, g, b);
    }

    public static void setBulkCachingMode(LynxModule.BulkCachingMode mode) {
        for (LynxModule module : allHubs)
            module.setBulkCachingMode(mode);
    }

    public static void clearBulkCaches() {
        for (LynxModule module : allHubs)
            module.clearBulkCache();
    }

    public static void FullInit(LinearOpMode OpMode) {
        forceInitRobot(opMode);
    }
}


