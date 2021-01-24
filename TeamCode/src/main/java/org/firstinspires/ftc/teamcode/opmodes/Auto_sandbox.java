package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2D;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;


@Autonomous
public class Auto_sandbox extends AutonomousOpMode {
    @Override
    public void main() {
        movement.Pos(new Pose2D(Double.NaN,Double.NaN,Math.toRadians(90)));
        movement.Pos(new Pose2D(Double.NaN,Double.NaN,Math.toRadians(180)));
        movement.Pos(new Pose2D(Double.NaN,Double.NaN,Math.toRadians(-90)));
        movement.Pos(new Pose2D(Double.NaN,Double.NaN,Math.toRadians(0)));
    }
}
