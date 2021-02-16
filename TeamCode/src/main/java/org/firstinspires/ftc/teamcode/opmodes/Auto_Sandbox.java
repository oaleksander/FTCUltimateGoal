package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.Pose2D;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;


@Autonomous
public class Auto_Sandbox extends AutoOpMode {
    @Override
    public void main() {
        movement.pos(new Pose2D(50,-100, toRadians(0)));
        movement.pos(new Pose2D(50, 100, toRadians(180)));
        movement.pos(new Pose2D(50, -100, toRadians(180)));
        movement.pos(new Pose2D(50, -100, toRadians(0)));
        movement.pos(getStartPosition());
        while (opModeIsActive()) {
            delay(1);
        }
    }
}
