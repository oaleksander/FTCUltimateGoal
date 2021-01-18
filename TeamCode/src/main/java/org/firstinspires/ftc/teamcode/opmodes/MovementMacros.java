package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator2;
import org.firstinspires.ftc.teamcode.robot.rpm;
import org.firstinspires.ftc.teamcode.superclasses.MotionTask;

import java.util.ArrayList;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.delay;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.getOpMode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.opMode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.spinOnce;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator2;

public class MovementMacros {

    static byte xSign = 1;
    static byte sideSign = 1;

    public MovementMacros(byte xSign, byte sideSign) {
        MovementMacros.xSign = xSign;
        MovementMacros.sideSign = sideSign;
    }

    public static void ShootTargets() {
        //shooter.setShootersetings(3850, 500);
        shooter.setShootingMode(rpm.ShooterMode.HIGHGOAL);
        if (sideSign == 1 && xSign == 1)
            movement.Pos(new Pose2D(xSign * 121, -48.5, toRadians(-10.5)));
        else if (sideSign == -1 && xSign == 1)
            movement.Pos(new Pose2D(xSign * 53, -30, toRadians(3)));
        else if (sideSign == -1 && xSign == -1)
            movement.Pos(new Pose2D(xSign * 147.5, -9.5, toRadians(10.5)));
        else // if (sideSign == 1 &&  xSign == -1)
            movement.Pos(new Pose2D(xSign * 61.5, -28, toRadians(-11.5)));
        ElapsedTime shooterAccelerationTimeout = new ElapsedTime();
        while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds()<3)
            spinOnce();
        delay(300);
        shooter.feedRings();
        delay(700);
        shooter.setShootingMode(rpm.ShooterMode.OFF);
    }
    public static void ShootPOWERSHOT() {  //rename
        movement.Pos(new Pose2D(Double.NaN, -15, Double.NaN));
        shooter.setShootingMode(rpm.ShooterMode.POWERSHOT);
        double pos = 27;
        double angle = 5.5;
        for (short i = 0; i < 3; i++){
           if (xSign == 1) {
               movement.Pos(new Pose2D(xSign * pos, -5, toRadians(angle)));
           }
           else {
               movement.Pos(new Pose2D(xSign * pos, -5, toRadians(3)));
           }
           angle -= 6.4;
           if(WoENrobot.getOpMode().gamepad1.x) {
               break;
           }
           //pos -= 18;
           // ElapsedTime shooterAccelerationTimeout = new ElapsedTime();
           // while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds()<3)
            //    spinOnce();
            shooter.feedRing();
            delay(200);
        }
        shooter.setShootingMode(rpm.ShooterMode.OFF);
    }
    public static void MoveWobble() {
        wobbleManipulator2.setposclose(true);
        wobbleManipulator2.changepos(WobbleManipulator2.positions.medium);
        if (sideSign * xSign == 1)
            switch (openCVNode.getStackSize()) {
                case FOUR:
                    movement.Pos(new Pose2D(xSign * 154, 113, toRadians(-8 * xSign)));
                    break;
                case ONE:
                    movement.Pos(new Pose2D(xSign * 144, 63, toRadians(-45 * xSign)));
                    break;
                case ZERO:
                default:
                    movement.Pos(new Pose2D(xSign * 145, -15, toRadians(0 * xSign)));
            }
        else

            switch (openCVNode.getStackSize()) {
                case FOUR:
                    movement.Pos(new Pose2D(xSign * 110, 116, toRadians(50 * xSign)));
                    break;
                case ONE:
                    movement.Pos(new Pose2D(xSign * 54.5 + 8.5, 63 + 3 * xSign, toRadians(50 * xSign)));
                    break;
                case ZERO:
                default:
                    movement.Pos(new Pose2D(xSign * 120, 0, toRadians(50 * xSign)));
            }
        wobbleManipulator2.changepos(WobbleManipulator2.positions.down);
        WoENrobot.delay(800);
        wobbleManipulator2.setposclose(false);
        WoENrobot.delay(600);
    }

    private static Vector2D getWobblePose()
    {
        switch(openCVNode.getStackSize())
        {
            case FOUR:
                return new Vector2D(xSign * 150.3809, 150.2761);
            case ONE:
                return new Vector2D(xSign * 89.9835, 90.3346);
            case ZERO:
            default:
                return new Vector2D(xSign * 150.3809, 30.1596);
        }
    }

    private static final Vector2D wobblePlacementOffset = new Vector2D(11.8425,33.25);

    public static void MoveWobble_experimental() {
        wobbleManipulator2.setposclose(true);
        wobbleManipulator2.changepos(WobbleManipulator2.positions.medium);
        Vector2D wobblePose = getWobblePose();
        Vector2D error = (Vector2D)movement.getError(new Pose2D(wobblePose,Double.NaN));
        movement.followPath(new MotionTask(wobblePose.minus(wobblePlacementOffset.rotatedCW(error.acot())),error.acot(), ()->{
                    wobbleManipulator2.changepos(WobbleManipulator2.positions.down);
                    WoENrobot.delay(666);
                    wobbleManipulator2.setposclose(false);
                    WoENrobot.delay(666);}));
        while(movement.pathFollowerIsActive()&&getOpMode().opModeIsActive()) {spinOnce();}
    }

    private static final double yParkLine = 26.462;
    private static final double robotYbackLength = 29.85498;
    private static final double robotYfrontLength = 37.2;
    private static final double parkingTolerance = 10;

    public static void Park() {
        if(odometry.getRobotCoordinates().y>yParkLine) {
            movement.Pos(new Pose2D(89.6372*xSign+67.3092*sideSign, yParkLine+robotYbackLength-parkingTolerance, 0));
        }
        else {
            wobbleManipulator2.changepos(WobbleManipulator2.positions.medium);
            movement.Pos(new Pose2D(89.6372*xSign+67.3092*sideSign, yParkLine-robotYfrontLength+parkingTolerance, 0));
            delay(50);
        }
    }

    public static void PutRingsToLowGoal() {
        movement.Pos(new Pose2D((93.75 + 11.25 * xSign) * xSign, 150, toRadians(0)));
        wobbleManipulator2.setAngle(0.55);
        delay(1000);
        wobbleManipulator2.setAngle(0.5);
    }

}
