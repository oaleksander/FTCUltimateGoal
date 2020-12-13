package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator2;

import static org.firstinspires.ftc.teamcode.MovementMacros.MoveWobble;
import static org.firstinspires.ftc.teamcode.MovementMacros.PutRingsToLowGoal;
import static org.firstinspires.ftc.teamcode.MovementMacros.ShootTargets;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;

@Autonomous
public class AutoUniversal extends AutonomousOpMode {

    @Override
    public void main() {
        delay(0);
        ShootTargets();
        MoveWobble();
        //movement.Pos(new Pose2D(getStartPosition().x, 25, 0));
        if(xSign*sideSign == 1)
            movement.Pos(new Pose2D(odometry.getRobotCoordinates().x+20*xSign, 0, 0));
        else
        movement.Pos(new Pose2D(odometry.getRobotCoordinates().x, 25, 0));
        wobbleManipulator2.changepos(WobbleManipulator2.positions.down);
        while(opModeIsActive()) {delay(1);}
    }
}
