package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;

@Autonomous(name = "Pos test")
public class TestPos extends AutonomousOpMode {
    @Override
    public void main() {
        while (opModeIsActive()) {
            openCVNode.retrieveResult();
            //telemetry.addData("rpm2", shooter.rpm2);
            shooter.onshooter(gamepad1.a);
            spinOnce();
        }
    }
}
