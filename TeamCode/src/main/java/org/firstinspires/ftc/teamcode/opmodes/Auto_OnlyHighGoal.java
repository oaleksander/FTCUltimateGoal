package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator;

import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.Park;
import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootHighGoal;
import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootTargets;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator;

@Autonomous
public class Auto_OnlyHighGoal extends AutoOpMode {

    @Override
    public void main() {
        movement.Pos(new Pose2D(odometry.getRobotCoordinates().x + 15 * sideSign, -50, Double.NaN), 1, 1, 10, 0.1);
        ShootHighGoal();
        // MoveWobble();
        //movement.Pos(new Pose2D(getStartPosition().x, 25, 0));
        Park();
        wobbleManipulator.setAngle(WobbleManipulator.Position.UP);
    }
}
