package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.park
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement

@Autonomous
class Auto_JustPark : AutoOpMode() {
    override fun main() {
        park()
    }
}