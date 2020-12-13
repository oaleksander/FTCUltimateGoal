package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator2;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.delay;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.opMode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator2;

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
        if (sideSign * xSign == 1)
            movement.Pos(new Pose2D(xSign * 128, -44, toRadians(-11 * xSign)));
        else
            movement.Pos(new Pose2D(xSign * 45, -48, toRadians(7 * xSign)));
        while (opMode.opModeIsActive() && !shooter.isCorrectRpm()) {
        }
        conveyor.feedrings();
        delay(1000);
        shooter.onshooter(false);
    }

    public static void MoveWobble() {
        wobbleManipulator2.setposclose(true);
        wobbleManipulator2.changepos(WobbleManipulator2.positions.medium);
        if (sideSign * xSign == 1)
            switch (openCVNode.getStackSize()) {
                case FOUR:
                    movement.Pos(new Pose2D(xSign * 154, 103, toRadians(-8 * xSign)));
                    break;
                case ONE:
                    movement.Pos(new Pose2D(xSign * 144, 60, toRadians(-60 * xSign)));
                    break;
                case ZERO:
                default:
                    movement.Pos(new Pose2D(xSign * 135, 35, toRadians(30 * xSign)));
            }
            else

        switch (openCVNode.getStackSize()) {
            case FOUR:
                movement.Pos(new Pose2D(xSign * 110, 116, toRadians(50 * xSign)));
                break;
            case ONE:
                movement.Pos(new Pose2D(xSign * 66, 61, toRadians(50 * xSign)));
                break;
            case ZERO:
            default:
                movement.Pos(new Pose2D(xSign * 120, 0, toRadians(50 * xSign)));
        }
        wobbleManipulator2.changepos(WobbleManipulator2.positions.down);
        WoENrobot.delay(1800);
        wobbleManipulator2.setposclose(false);
        WoENrobot.delay(300);
    }

    public static void PutRingsToLowGoal() {
        movement.Pos(new Pose2D((93.75 + 11.25 * xSign) * xSign, 150, toRadians(0)));
        wobbleManipulator2.setAngle(0.55);
        delay(1000);
        wobbleManipulator2.setAngle(0.5);
    }
}
