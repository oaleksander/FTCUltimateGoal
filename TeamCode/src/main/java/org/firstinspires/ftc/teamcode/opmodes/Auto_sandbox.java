package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.superclasses.MotionTask;
import org.openftc.revextensions2.ExpansionHubServo;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;


@Autonomous
public class Auto_sandbox extends AutonomousOpMode {
    @Override
    public void main() {
        ElapsedTime et = new ElapsedTime();
        while(opModeIsActive())
        drivetrain.setRobotVelocity(Math.sin(et.seconds()*4*Math.PI)*120,0,0);
        spinOnce();
    }
}
