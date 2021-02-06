package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.*;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;

@Autonomous
public class Auto_TwoWobble extends AutoOpMode {

    @Override
    public void main() {
        conveyor.setAutomaticConveyorStopping(true);
        movement.Pos(new Pose2D(odometry.getRobotCoordinates().x + 15 * sideSign, -50, Double.NaN), 1, 1, 10, 0.1);
        Shooting();
        MoveWobble();
        PickupRings();
        if (WoENrobot.runTime.seconds() < 23) {
            PickSecondWobble();
            MoveWobble();
        }
        Park();
    }
}
//Shooting();
