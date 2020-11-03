package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MovementMacros;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;

import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import org.firstinspires.ftc.teamcode.superclasses.BlueLeftAutonomous;

import static java.lang.Math.toRadians;

@Autonomous(name = "Auto Blue Left")
public class Auto_BlueLeft extends BlueLeftAutonomous {

    @Override
    public void main() {
      MovementMacros.MoveWobble(getXSign());
      drivetrain.Pos(new Pose2D(-125, 25, toRadians(180)));
      wobbleManipulator.setposlever(360);
    }
}
