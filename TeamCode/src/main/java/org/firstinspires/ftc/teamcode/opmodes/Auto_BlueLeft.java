package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.OpenCVNode.StackSize;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import org.firstinspires.ftc.teamcode.superclasses.BlueLeftAutonomous;

import static java.lang.Math.toRadians;

@Autonomous(name = "Auto Blue Left")
public class Auto_BlueLeft extends BlueLeftAutonomous {

    @Override
    public void mainProgram() {
        WoENrobot.wobbleManipulator.setposclose(true);
        WoENrobot.wobbleManipulator.setposlever(0);
        WoENrobot.drivetrain.Pos(new Pose2D(-93.75, -142, toRadians(180)));
        WoENrobot.delay(2000);
        switch (WoENrobot.openCVNode.retrieveResult()) {
            case FOUR:
                WoENrobot.drivetrain.Pos(new Pose2D(-150, 126, toRadians(180)));
                break;
            case ONE:
                WoENrobot.drivetrain.Pos(new Pose2D(-91, 66, toRadians(180)));
                break;
            case ZERO:
            default:
                WoENrobot.drivetrain.Pos(new Pose2D(-150, 6, toRadians(180)));
        }
        WoENrobot.wobbleManipulator.setposlever(780);
        WoENrobot.delay(1200);
        WoENrobot.wobbleManipulator.setposclose(false);
        WoENrobot.delay(500);
        WoENrobot.drivetrain.Pos(new Pose2D(-125, 25, toRadians(180)));
        WoENrobot.wobbleManipulator.setposlever(360);
    }
}
