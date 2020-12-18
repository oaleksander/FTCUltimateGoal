package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.superclasses.MotionTask;
import org.openftc.revextensions2.ExpansionHubServo;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;


@Autonomous
public class Auto_sandbox extends AutonomousOpMode {
    @Override
    public void main() {
        shooter.setShootersetings(3800,1);
        shooter.onshooter(true);
        do {
            delay(1);
        } while (opModeIsActive());
        shooter.onshooter(false);
    }
}
