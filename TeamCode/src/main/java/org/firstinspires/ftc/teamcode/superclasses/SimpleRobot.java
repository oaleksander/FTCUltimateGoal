package org.firstinspires.ftc.teamcode.superclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.OpenCVNode;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

public class SimpleRobot {
    public static LinearOpMode opMode = null;
    protected static RobotModule[] activeAobotModules;

    public static boolean robotIsInitialized = false;

    public static ElapsedTime Runtime = new ElapsedTime();

    public static void delay(double delay_ms)
    {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds()<delay_ms && opMode.opModeIsActive());
    }

    public static void startRobot() {
        opMode.waitForStart();
        regulatorUpdater.start();
        Runtime.reset();
        opMode.telemetry.addData("Status", "Running");
        opMode.telemetry.update();
    }

    public static void initRobot(LinearOpMode opMode) {
        if (!robotIsInitialized) {
            forceInitRobot(opMode);
            opMode.telemetry.addData("Status", "Initialization successful");
            opMode.telemetry.update();
        } else {
            WoENrobot.opMode = opMode;
            for(RobotModule robotModule : activeAobotModules)
            {
                robotModule.setOpMode(opMode);
            }
            if (regulatorUpdater.getState() != Thread.State.NEW)
            {
                regulatorUpdater = new Thread(updateRegulators);
            }
            opMode.telemetry.addData("Status", "Already initialized, ready");
            opMode.telemetry.update();
        }
    }

    private static final boolean defaultNames = true;

    public static void forceInitRobot(LinearOpMode opMode) {
        WoENrobot.opMode = opMode;
        for(RobotModule robotModule : activeAobotModules)
        {
            robotModule.initialize(opMode);
        }

        opMode.telemetry.addData("Status", "Initializing...");
        opMode.telemetry.update();

        if (regulatorUpdater.getState() != Thread.State.NEW)
        {
            regulatorUpdater = new Thread(updateRegulators);
        }
        //stopAllMotors();

        robotIsInitialized = true;
        opMode.telemetry.addData("Status", "Force initialized");
        opMode.telemetry.update();
    }

    public static void SimpleInit(LinearOpMode opMode) {
        WoENrobot.initRobot(opMode);
        WoENrobot.startRobot();
    }
    public static void FullInit(LinearOpMode opMode) {
        WoENrobot.forceInitRobot(opMode);
        WoENrobot.startRobot();
    }


    static Runnable updateRegulators = () -> {
        while(opMode.opModeIsActive()) {
            for(RobotModule robotModule : activeAobotModules)
            {
                robotModule.update();
            }
        }
        opMode.telemetry.update();
    };

    private static Thread regulatorUpdater = new Thread(updateRegulators);
}
