package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.delay;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator2;

public class MovementMacros {

    static byte xSign = 1;
    static byte sideSign = 1;

    public MovementMacros(byte xSign, byte sideSign) {
        MovementMacros.xSign = xSign;
        MovementMacros.sideSign = sideSign;
    }

    public static void MoveWobble() {
        wobbleManipulator2.setposclose(true);
        wobbleManipulator2.setAngle(0);
        switch (openCVNode.getStackSize()) {
            case FOUR:
                movement.Pos(new Pose2D(xSign * 150, 126, toRadians(0)));
                break;
            case ONE:
                movement.Pos(new Pose2D(xSign * 91, 66, toRadians(0)));
                break;
            case ZERO:
            default:
                movement.Pos(new Pose2D(xSign * 150, 6, toRadians(0)));
        }
        wobbleManipulator2.setposclose(false);
        WoENrobot.wobbleManipulator2.setAngle(0.25);
        WoENrobot.delay(100);
        WoENrobot.wobbleManipulator2.setAngle(0);
        WoENrobot.delay(100);
        WoENrobot.wobbleManipulator2.setAngle(0.25);
    }

    public static void PutRingsToLowGoal() {
        movement.Pos(new Pose2D((93.75 + 11.25 * xSign) * xSign, 150, toRadians(0)));
        wobbleManipulator2.setAngle(0.55);
        delay(1000);
        wobbleManipulator2.setAngle(0.5);
    }
}
