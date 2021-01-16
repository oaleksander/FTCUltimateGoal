package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator2;
import org.firstinspires.ftc.teamcode.robot.rpm;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.delay;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
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
         //  if(gamepad1.x) {
         //      break;
         //  }
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

    public static void PutRingsToLowGoal() {
        movement.Pos(new Pose2D((93.75 + 11.25 * xSign) * xSign, 150, toRadians(0)));
        wobbleManipulator2.setAngle(0.55);
        delay(1000);
        wobbleManipulator2.setAngle(0.5);
    }
}
