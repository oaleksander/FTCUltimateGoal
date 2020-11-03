package org.firstinspires.ftc.teamcode.robot;

import android.app.ExpandableListActivity;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

public class WoENrobot {
    public static TwoWheelOdometry odometry = new TwoWheelOdometry();
    public static Drivetrain drivetrain = new Drivetrain(odometry);
    public static WobbleManipulator wobbleManipulator = new WobbleManipulator();
    public static OpenCVNode openCVNode = new OpenCVNode();
    public static Conveyor conveyor = new Conveyor();
    public static rpm shooter = new rpm();
    protected static RobotModule[] activeAobotModules = {conveyor, odometry, shooter, wobbleManipulator, drivetrain};

    public static void FullInitWithCV(LinearOpMode opMode) {
        forceInitRobot(opMode);
        openCVNode.initialize(opMode);
        startRobot();
    }

    private static final boolean defaultNames = true;
    public static LinearOpMode opMode = null;
    public static boolean robotIsInitialized = false;

    private static ExpansionHubEx expansionHub1 = null;
    private static ExpansionHubEx expansionHub2 = null;

    public static ElapsedTime Runtime = new ElapsedTime();
    public static ElapsedTime looptime = new ElapsedTime();
    static Runnable updateRegulators = () -> {
        while (opMode.opModeIsActive()) {
            for (RobotModule robotModule : activeAobotModules) {
                robotModule.update();
            }
            opMode.telemetry.addData("Loop frequency", 1.0/looptime.seconds() + " hz");
            looptime.reset();
            spinCompleted = true;
            opMode.telemetry.update();
        }
    };
    private static Thread regulatorUpdater = new Thread(updateRegulators);

    static boolean spinCompleted = false;
    public static void spinOnce()
    {
    spinCompleted = false;
    while(!spinCompleted && opMode.opModeIsActive()) {}
    }
    public static void delay(double delay_ms) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < delay_ms && opMode.opModeIsActive()) ;
    }

    public static void startRobot() {
        setLedColors(255,166,0);
        opMode.waitForStart();
        regulatorUpdater.start();
        Runtime.reset();
        setLedColors(0,237,255);
        opMode.telemetry.addData("Status", "Running");
        opMode.telemetry.update();
    }

    public static void initRobot(LinearOpMode opMode) {
        if (!robotIsInitialized) {
            forceInitRobot(opMode);
            opMode.telemetry.addData("Status", "Initialization successful");
            opMode.telemetry.update();
        } else {
            org.firstinspires.ftc.teamcode.robot.WoENrobot.opMode = opMode;
            for (RobotModule robotModule : activeAobotModules) {
                robotModule.setOpMode(opMode);
            }
            if (regulatorUpdater.getState() != Thread.State.NEW) {
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
            regulatorUpdater = new Thread(updateRegulators);
        }
        //stopAllMotors();

        robotIsInitialized = true;

        opMode.telemetry.addData("Status", "Force initialized");
        opMode.telemetry.update();

    }

    public static void SimpleInit(LinearOpMode OpMode) {
        initRobot(opMode);
        startRobot();
    }

    public static void setLedColors(int r, int g, int b)
    {
        expansionHub1.setLedColor(r, g, b);
        expansionHub2.setLedColor(r, g, b);
    }

    public static void FullInit(LinearOpMode OpMode) {
        forceInitRobot(opMode);
        startRobot();
    }
}


