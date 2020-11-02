package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.math.Pose2D;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.drivetrain;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator;
import static org.firstinspires.ftc.teamcode.superclasses.SimpleRobot.delay;

public class MovementMacros {
    public static void MoveWobble(byte Xsign)
    {
        wobbleManipulator.setposclose(true);
        wobbleManipulator.setposlever(0);
        switch (openCVNode.retrieveResult()) {
            case FOUR:
                drivetrain.Pos(new Pose2D(Xsign*150, 126, toRadians(0)));
                break;
            case ONE:
                drivetrain.Pos(new Pose2D(Xsign*150, 66, toRadians(0)));
                break;
            case ZERO:
            default:
                drivetrain.Pos(new Pose2D(Xsign*150, 6, toRadians(0)));
        }
        wobbleManipulator.setposlever(780);
        delay(1200);
        wobbleManipulator.setposclose(false);
        delay(500);
    }
}
