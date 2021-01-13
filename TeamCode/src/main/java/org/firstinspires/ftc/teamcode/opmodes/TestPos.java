package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootPOWERSHOT;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.spinOnce;

@Autonomous(name = "Pos test")
public class TestPos extends AutonomousOpMode {
    @Override
    public void main() {
        ShootPOWERSHOT();
    }
}
