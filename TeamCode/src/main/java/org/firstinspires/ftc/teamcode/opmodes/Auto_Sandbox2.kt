package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.shootPOWERSHOTAngle

@Autonomous(name = "Pos test")
class Auto_Sandbox2 : AutoOpMode() {
    override fun main() {
        shootPOWERSHOTAngle()
    }
}