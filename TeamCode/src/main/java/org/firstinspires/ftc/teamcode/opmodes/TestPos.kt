package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootPOWERSHOTAngle

@Autonomous(name = "Pos test")
class TestPos : AutoOpMode() {
    override fun main() {
        ShootPOWERSHOTAngle()
    }
}