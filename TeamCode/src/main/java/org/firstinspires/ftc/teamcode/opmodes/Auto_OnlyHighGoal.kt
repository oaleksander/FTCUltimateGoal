package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.Park
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootHighGoal
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry
import org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator

@Autonomous
class Auto_OnlyHighGoal : AutoOpMode() {
    override fun main() {
        movement.pos(Pose2D(odometry.robotCoordinates.x + 15 * sideSign, -50.0, Double.NaN), 1.0, 1.0, 10.0, 0.1)
        ShootHighGoal()
        // MoveWobble();
        //movement.Pos(new Pose2D(getStartPosition().x, 25, 0));
        Park()
        wobbleManipulator.setAngle(WobbleManipulator.Position.UP)
    }
}