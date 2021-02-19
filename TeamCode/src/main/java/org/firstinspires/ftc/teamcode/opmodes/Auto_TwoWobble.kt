package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MoveWobble
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.Park
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.PickSecondWobble
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.PickupRings
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.Shooting
import org.firstinspires.ftc.teamcode.robot.WoENrobot.*

@Autonomous
class Auto_TwoWobble : AutoOpMode() {
    override fun main() {
        conveyor.setAutomaticConveyorStopping(true)
        movement.pos(Pose2D(odometry.robotCoordinates.x + 15 * sideSign, -50.0, Double.NaN), 1.0, 1.0, 10.0, 0.1)
        Shooting()
        MoveWobble()
        PickupRings()
        if (runTime.seconds() < 23) {
            PickSecondWobble()
            MoveWobble()
        }
        Park()
    }
}