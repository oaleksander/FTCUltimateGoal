package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator2;

import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MoveWobble;
import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootTargets;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.delay;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator2;

@Autonomous
public class AutoUniversal extends AutonomousOpMode {

    @Override
    public void main() {
        delay(0);
        ShootTargets();
        MoveWobble();
        //movement.Pos(new Pose2D(getStartPosition().x, 25, 0));
        if (xSign * sideSign == 1)
            movement.Pos(new Pose2D(odometry.getRobotCoordinates().x + 20 * xSign, 0, 0));
        else
            movement.Pos(new Pose2D(odometry.getRobotCoordinates().x, 25, 0));
        wobbleManipulator2.changepos(WobbleManipulator2.positions.down);
        delay(3000);
    }
}
