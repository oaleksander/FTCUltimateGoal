package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.superclasses.Odometry;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.firstinspires.ftc.teamcode.superclasses.SimpleRobot;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.signum;

public class WoENrobot extends SimpleRobot {

    private WoENrobot() {
        activeAobotModules = new RobotModule[]{odometry, drivetrain, wobbleManipulator};
    }
    public static Odometry odometry = new TwoWheelOdometry();
    public static Drivetrain drivetrain = new Drivetrain(odometry);
    public static WobbleManipulator wobbleManipulator = new WobbleManipulator();

}

