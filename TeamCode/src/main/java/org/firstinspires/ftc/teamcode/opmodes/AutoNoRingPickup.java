package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.Pose2D;

import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MoveWobble;
import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.Park;
import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootHighGoal;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;

@Autonomous
public class AutoNoRingPickup extends AutonomousOpMode {

    @Override
    public void main() {
        movement.Pos(new Pose2D(odometry.getRobotCoordinates().x+10*sideSign, -60, Double.NaN));
        ShootHighGoal();
       // delay(3000);
        MoveWobble();
        Park();
    }
}
//Shooting();
