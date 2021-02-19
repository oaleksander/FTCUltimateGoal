package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.Pose2D;

import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.MoveWobble;
import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.Park;
import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootHighGoal;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;

@Autonomous
public class Auto_NoRingPickup extends AutoOpMode {

    @Override
    public void main() {
        movement.pos(new Pose2D(odometry.getRobotCoordinates().x + 10 * sideSign, -60, Double.NaN));
        ShootHighGoal(); //TODO Powershots
        // delay(3000);
        MoveWobble();
        Park();
    }
}
//Shooting();
