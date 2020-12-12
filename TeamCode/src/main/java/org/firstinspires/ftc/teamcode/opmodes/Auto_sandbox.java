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
        do {
            movement.Pos(new Pose2D(getStartPosition().x, 25, 0));
            MotionTask mt = new MotionTask(0,0,0,()->{});
            Pose2D p2d = (Pose2D)mt;
        } while (opModeIsActive());
    }
}
