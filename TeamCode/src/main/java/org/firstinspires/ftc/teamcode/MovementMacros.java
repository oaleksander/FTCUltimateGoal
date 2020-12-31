package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator2;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;

public class MovementMacros {

    static byte xSign = 1;
    static byte sideSign = 1;

    public MovementMacros(byte xSign, byte sideSign) {
        MovementMacros.xSign = xSign;
        MovementMacros.sideSign = sideSign;
    }

    public static void ShootTargets() {
        shooter.setShootersetings(3850, 500);
        shooter.onshooter(true);
        if (sideSign == 1 &&  xSign == 1)
            movement.Pos(new Pose2D(xSign * 121, -48.5, toRadians(-10.5)));
        else
        if (sideSign == -1 &&  xSign == 1)
            movement.Pos(new Pose2D(xSign * 53, -30, toRadians(3)));
        else
        if (sideSign == -1 &&  xSign == -1)
            movement.Pos(new Pose2D(xSign * 147.5, -9.5, toRadians(10.5)));
        else // if (sideSign == 1 &&  xSign == -1)
            movement.Pos(new Pose2D(xSign * 61.5, -28, toRadians(-11.5)));
        while (opMode.opModeIsActive() && !shooter.isCorrectRpm()) { }
        conveyor.feedrings();
        delay(900);
        shooter.onshooter(false);
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
                movement.Pos(new Pose2D(xSign * 54.5+8.5, 63+3*xSign, toRadians(50 * xSign)));
                break;
            case ZERO:
            default:
                movement.Pos(new Pose2D(xSign * 120, 0, toRadians(50 * xSign)));
        }
        wobbleManipulator2.changepos(WobbleManipulator2.positions.down);
        WoENrobot.delay(1800);
        wobbleManipulator2.setposclose(false);
        WoENrobot.delay(1000);
    }

    public static void PutRingsToLowGoal() {
        movement.Pos(new Pose2D((93.75 + 11.25 * xSign) * xSign, 150, toRadians(0)));
        wobbleManipulator2.setAngle(0.55);
        delay(1000);
        wobbleManipulator2.setAngle(0.5);
    }
}
