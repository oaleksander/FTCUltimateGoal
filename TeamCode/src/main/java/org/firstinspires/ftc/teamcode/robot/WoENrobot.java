package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.Arrays;
import java.util.List;

public class WoENrobot {

    public static ServoWobbleManipulator wobbleManipulator = new ServoWobbleManipulator();
    public static OpenCVNodeWebcam openCVNode = new OpenCVNodeWebcam();
    public static Conveyor2 conveyor = new Conveyor2();
    public static rpm shooter = new rpm();
    public static TelemetryDebugging telemetryDebugging = new TelemetryDebugging();
    public static AI ai = new AI();

    //public static FakeRobot odometry = new FakeRobot();
    //public static FakeRobot drivetrain = odometry;

    public static ThreeWheelOdometry odometry = new ThreeWheelOdometry();
    public static MecanumDrivetrain drivetrain = new MecanumDrivetrain();
    public static Movement movement = new Movement(odometry, drivetrain);
    public static LinearOpMode opMode = null;
    public static boolean robotIsInitialized = false;
    public static final ElapsedTime runTime = new ElapsedTime();
    protected static MultithreadRobotModule[] activeRobotModules = {odometry, movement, drivetrain, wobbleManipulator, conveyor, shooter, telemetryDebugging, ai}; //
    static volatile boolean controlHubSpinCompleted = false;
    static volatile boolean expansionHubSpinCompleted = false;
    static volatile boolean spinCompleted = false;
    public static ExpansionHubEx controlHub = null;
    public static ExpansionHubEx expansionHub = null;
    private static List<LynxModule> allHubs = null;

    static Runnable updateControlHub = () -> {
        controlHub.getStandardModule().setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
         while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted()) {
             controlHub.getStandardModule().clearBulkCache();
             Arrays.stream(activeRobotModules).forEach(MultithreadRobotModule::updateControlHub);
             controlHubSpinCompleted = true;
         }
    };
    private static Thread controlHubUpdater = new Thread(updateControlHub);
    static Runnable updateExpansionHub = () -> {
        expansionHub.getStandardModule().setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            expansionHub.getStandardModule().clearBulkCache();
            Arrays.stream(activeRobotModules).forEach(MultithreadRobotModule::updateExpansionHub);
            expansionHubSpinCompleted = true;
        }
    };
    private static Thread expansionHubUpdater = new Thread(updateExpansionHub);
    static Runnable updateOther = () -> {
        while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            while ((!controlHubSpinCompleted || !expansionHubSpinCompleted) && opMode.opModeIsActive()) {
                Thread.yield();
            }
            Arrays.stream(activeRobotModules).forEach(MultithreadRobotModule::updateOther);
            controlHubSpinCompleted = false;
            expansionHubSpinCompleted = false;
            spinCompleted = true;
        }
    };
    private static Thread otherUpdater = new Thread(updateOther);

    static Runnable updateRegulators = () -> {
        setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            clearBulkCaches();
            Arrays.stream(activeRobotModules).forEach(MultithreadRobotModule::update);
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

    private static synchronized void spinHubs() {
        controlHubSpinCompleted = false;
        expansionHubSpinCompleted = false;
            while ((!controlHubSpinCompleted || !expansionHubSpinCompleted) && opMode.opModeIsActive()) {
                Thread.yield();
            }
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
        if(opMode.isStopRequested()) return;
        runTime.reset();
        Arrays.stream(activeRobotModules).forEach(MultithreadRobotModule::start);
       // regulatorUpdater.start();
        controlHubUpdater.start();
        expansionHubUpdater.start();
        otherUpdater.start();
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
            Arrays.stream(activeRobotModules).forEach(robotModule -> robotModule.setOpMode(opMode));
           // regulatorUpdater.interrupt();
          //  regulatorUpdater = new Thread(updateRegulators);

            opMode.telemetry.addData("Status", "Already initialized, ready");
            opMode.telemetry.update();
        }
    }

    public static void forceInitRobot(LinearOpMode OpMode) {
        opMode = OpMode;

        opMode.telemetry.addData("Status", "Initializing...");
        opMode.telemetry.update();


        WoENHardware.INSTANCE.assignHardware(opMode.hardwareMap);
        controlHub = WoENHardware.INSTANCE.getControlHub();
        expansionHub = WoENHardware.INSTANCE.getExpansionHub();
        allHubs = WoENHardware.INSTANCE.getLynxModules();

        setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        Arrays.stream(activeRobotModules).forEach(robotModule -> robotModule.initialize(opMode));

        if (regulatorUpdater.getState() != Thread.State.NEW) {
            regulatorUpdater.interrupt();
            regulatorUpdater = new Thread(updateRegulators);
        }

        controlHubUpdater.interrupt();
        controlHubUpdater = new Thread(updateControlHub);
        expansionHubUpdater.interrupt();
        expansionHubUpdater = new Thread(updateExpansionHub);
        otherUpdater.interrupt();
        otherUpdater = new Thread(updateOther);
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
        controlHub.setLedColor(r, g, b);
        expansionHub.setLedColor(r, g, b);
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


