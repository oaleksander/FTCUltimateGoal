package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;

import static org.firstinspires.ftc.teamcode.MovementMacros.MoveWobble;
import static org.firstinspires.ftc.teamcode.MovementMacros.PutRingsToLowGoal;
import static org.firstinspires.ftc.teamcode.MovementMacros.ShootTargets;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;

@Autonomous
public class AutoUniversal extends AutonomousOpMode {

    @Override
    public void main() {
        AutoTransitioner.transitionOnStop(this, "TeleOp COMPETITION");
        MoveWobble();
        PutRingsToLowGoal();
        ShootTargets();
        movement.Pos(new Pose2D(getStartPosition().x, 25, 0));
        wobbleManipulator2.setAngle(0.2);
    }
}
