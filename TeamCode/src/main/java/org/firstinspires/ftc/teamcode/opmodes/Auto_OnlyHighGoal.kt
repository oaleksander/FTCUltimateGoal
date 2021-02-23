package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.avoidRingStack
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.park
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.shootHighGoal
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry
import org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator

@Autonomous
class Auto_OnlyHighGoal : AutoOpMode() {
    override fun main() {
        avoidRingStack()
        shootHighGoal()
        park()
    }
}