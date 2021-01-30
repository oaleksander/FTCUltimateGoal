package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator;

import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootTargets;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator;

@Autonomous
public class AutoNoWobble extends AutonomousOpMode {

    @Override
    public void main() {
        ShootTargets();
        // MoveWobble();
        //movement.Pos(new Pose2D(getStartPosition().x, 25, 0));
        movement.Pos(new Pose2D(odometry.getRobotCoordinates().x, 25, 0));
        wobbleManipulator.setAngle(WobbleManipulator.Position.UP);
    }
}
