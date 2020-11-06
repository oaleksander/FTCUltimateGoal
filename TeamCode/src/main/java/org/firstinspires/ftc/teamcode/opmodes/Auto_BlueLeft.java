package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MovementMacros;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;

import org.firstinspires.ftc.teamcode.superclasses.BlueLeftAutonomous;

import static java.lang.Math.toRadians;

@Autonomous(name = "Auto Blue Left")
public class

Auto_BlueLeft extends BlueLeftAutonomous {

    @Override
    public void main() {
    /*  shooter.setrpm(6000);
      shooter.setspeedlevel(5000);
      shooter.onshooter(true);
      do {
          telemetry.addData("rpm", shooter.rpm2);
          telemetry.addData("powercorect",shooter.power2);
        telemetry.addData("1",shooter.shooterMotor.getCurrentPosition());
      }while (true);
       */MovementMacros.MoveWobble(getXSign());
      drivetrain.Pos(new Pose2D(-125, 25, toRadians(180)));
      wobbleManipulator.setposlever(360);
    }
}
