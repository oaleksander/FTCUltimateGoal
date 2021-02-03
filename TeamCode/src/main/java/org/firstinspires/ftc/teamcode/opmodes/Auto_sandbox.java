package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2D;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;


@Autonomous
public class Auto_sandbox extends AutonomousOpMode {
    @Override
    public void main() {
        movement.Pos(new Pose2D(0,-100, Math.toRadians(0)));
        movement.Pos(new Pose2D(0, 100, Math.toRadians(180)));
        movement.Pos(new Pose2D(0, -100, Math.toRadians(180)));
        movement.Pos(new Pose2D(0, -100, Math.toRadians(0)));
        while (opModeIsActive()) {
            delay(1);
        }
    }
}
