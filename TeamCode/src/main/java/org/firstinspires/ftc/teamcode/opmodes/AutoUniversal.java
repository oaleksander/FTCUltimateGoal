package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator2;

import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MoveWobble;
import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootPOWERSHOTAngle;
import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootTargets;
import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.*;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.delay;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator2;

@Autonomous
public class AutoUniversal extends AutonomousOpMode {

    @Override
    public void main() {
        conveyor.setBackOnAfter(true);
        movement.Pos(new Pose2D(odometry.getRobotCoordinates().x+13*sideSign, -60, Double.NaN));
        ShootHighGoal();
       // delay(3000);
        MoveWobble_experimental();
        PickupRings();
        Park();
    }
}
//Shooting();
