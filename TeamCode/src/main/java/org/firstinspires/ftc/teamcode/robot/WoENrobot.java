package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.firstinspires.ftc.teamcode.superclasses.SimpleRobot;

public class WoENrobot extends SimpleRobot {

    public static TwoWheelOdometry odometry = new TwoWheelOdometry();
    public static Drivetrain drivetrain = new Drivetrain(odometry);
    public static WobbleManipulator wobbleManipulator = new WobbleManipulator();
    public static OpenCVNode openCVNode = new OpenCVNode();
    private WoENrobot() {
        activeAobotModules = new RobotModule[]{odometry, drivetrain, wobbleManipulator};
    }

    public static void FullInitWithCV(LinearOpMode opMode) {
        WoENrobot.forceInitRobot(opMode);
        openCVNode.initialize(opMode);
        WoENrobot.startRobot();
    }

}

