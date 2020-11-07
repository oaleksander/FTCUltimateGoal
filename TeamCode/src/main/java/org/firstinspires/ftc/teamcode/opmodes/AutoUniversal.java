package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.Conveyor;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import org.firstinspires.ftc.teamcode.robot.rpm;
import org.firstinspires.ftc.teamcode.superclasses.AutonomousOpMode;

import static org.firstinspires.ftc.teamcode.MovementMacros.MoveWobble;
import static org.firstinspires.ftc.teamcode.MovementMacros.PutRingsToLowGoal;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.drivetrain;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.spinOnce;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator;

@Autonomous
public class AutoUniversal extends AutonomousOpMode {

    @Override
    public void main() {
        AutoTransitioner.transitionOnStop(this, "TeleOp COMPETITION");
        MoveWobble();
        PutRingsToLowGoal();
        drivetrain.Pos(new Pose2D(getStartPosition().x, 25, 0));
        wobbleManipulator.setposlever(360);
    }
}
