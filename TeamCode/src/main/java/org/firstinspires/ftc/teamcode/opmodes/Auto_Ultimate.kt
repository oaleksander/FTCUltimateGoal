package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.avoidRingStack
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.moveWobble
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.park
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.pickSecondWobble
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.pickupRings
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.shooting
import org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry
import org.firstinspires.ftc.teamcode.robot.WoENrobot.runTime

@Autonomous
class Auto_Ultimate : AutoOpMode() {
    override fun main() {
        conveyor.setAutomaticConveyorStopping(true)
        conveyor.setReverseAfterStop(true)
        avoidRingStack()
        shooting()
        moveWobble()
        pickupRings()
        if (runTime.seconds() < 23) {
            pickSecondWobble()
            moveWobble()
        }
        park()
    }
}