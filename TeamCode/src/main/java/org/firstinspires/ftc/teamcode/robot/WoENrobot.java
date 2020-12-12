
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.openftc.revextensions2.ExpansionHubEx;

public class WoENrobot {

    public static WobbleManipulator2 wobbleManipulator2 = new WobbleManipulator2();
    public static OpenCVNode openCVNode = new OpenCVNode();
    public static Conveyor conveyor = new Conveyor();
    public static rpm shooter = new rpm();
    public static TelemetryDebugging telemetryDebugging = new TelemetryDebugging();

    public static ThreeWheelOdometry odometry = new ThreeWheelOdometry();
    public static MecanumDrivetrain drivetrain = new MecanumDrivetrain();
    public static Movement movement = new Movement(odometry,drivetrain);


    protected static RobotModule[] activeAobotModules = {odometry, drivetrain, movement, shooter, wobbleManipulator2, conveyor, telemetryDebugging}; //conveyor, odometry, shooter, wobbleManipulator, drivetrain

    public static LinearOpMode opMode = null;
    public static boolean robotIsInitialized = false;
    public static ElapsedTime runTime = new ElapsedTime();
    static boolean spinCompleted = false;
    static Runnable updateRegulators = () -> {
        while (opMode.opModeIsActive()&&!Thread.interrupted()) {

            for (RobotModule robotModule : activeAobotModules) {
                robotModule.update();
            }
            spinCompleted = true;
            opMode.telemetry.update();
        }
    };
    private static ExpansionHubEx expansionHub1 = null;
    private static ExpansionHubEx expansionHub2 = null;
    private static Thread regulatorUpdater = new Thread(updateRegulators);

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
            for (RobotModule robotModule : activeAobotModules) {
                robotModule.setOpMode(opMode);
            }
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


        expansionHub1 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHub2 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        for (RobotModule robotModule : activeAobotModules) {
            robotModule.initialize(opMode);
        }

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

    public static void FullInit(LinearOpMode OpMode) {
        forceInitRobot(opMode);
    }
}


