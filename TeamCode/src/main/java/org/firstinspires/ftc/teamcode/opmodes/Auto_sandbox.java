package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2D;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;


@Autonomous
public class Auto_sandbox extends AutonomousOpMode {
    @Override
    public void main() {
        ElapsedTime et = new ElapsedTime();
        while(opModeIsActive()) {
            movement.Pos(new Pose2D(Double.NaN, 120, Double.NaN));
            movement.Pos(new Pose2D(-120, Double.NaN, Double.NaN));
            movement.Pos(new Pose2D(Double.NaN, -120, Double.NaN));
            movement.Pos(new Pose2D(120, Double.NaN, Double.NaN));
        }
    }
}
