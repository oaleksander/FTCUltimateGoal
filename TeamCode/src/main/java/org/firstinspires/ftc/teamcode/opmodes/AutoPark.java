package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;

@Autonomous
public class AutoPark extends AutonomousOpMode {

    @Override
    public void main() {
        // MoveWobble();
        //PutRingsToLowGoal();
        movement.Pos(new Pose2D(getStartPosition().x, 25, 0));
        //wobbleManipulator.setposlever(360);
    }
}
