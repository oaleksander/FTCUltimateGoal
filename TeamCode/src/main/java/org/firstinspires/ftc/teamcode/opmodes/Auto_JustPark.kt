package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement

@Autonomous
class Auto_JustPark : AutoOpMode() {
    override fun main() {
        // MoveWobble();
        //PutRingsToLowGoal();
        movement.pos(Pose2D(startPosition.x, 25.0, 0.0))
    //wobbleManipulator.setposlever(360);
    }
}