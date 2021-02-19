package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MoveWobble
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.Park
import org.firstinspires.ftc.teamcode.robot.WoENrobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement

@Autonomous
class Auto_NoRingPickup : AutoOpMode() {
    override fun main() {
        movement.pos(Pose2D(WoENrobot.odometry.robotCoordinates.x + 10 * sideSign, -60.0, Double.NaN))
        MovementMacros.ShootHighGoal()
        // delay(3000);
        MoveWobble()
        Park()
    }
}