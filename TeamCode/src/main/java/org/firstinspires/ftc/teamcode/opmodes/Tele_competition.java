package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.misc.ButtonSwitch;
import org.firstinspires.ftc.teamcode.misc.SinglePressButton;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.rpm;

import static org.firstinspires.ftc.teamcode.opmodes.MovementMacros.ShootPOWERSHOTAngle;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.drivetrain;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.initRobot;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.spinOnce;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator2;

@TeleOp(name = "TeleOp COMPETITION", group = "Competition")
public class Tele_competition extends LinearOpMode {
    @Override
    public void runOpMode() {
        initRobot(this);
        startRobot();
      //  conveyor.setBackOnAfter(true);
        ButtonSwitch buttonAswitch = new ButtonSwitch();
        ButtonSwitch buttonBackswitch = new ButtonSwitch();
        ButtonSwitch shooterSpeedSwitch = new ButtonSwitch();
        //  ButtonSwitch buttonspeedConveyor = new ButtonSwitch();

        ButtonSwitch shooterOnOffSwitch = new ButtonSwitch();
        SinglePressButton threeRingPresser = new SinglePressButton();
        //conveyor.feeder.setPosition(0.06);
        //  double powerconveyor = 1;

        //shooter.setShootersetings(3800, 500);
        while (opModeIsActive()) {
        /*    if (gamepad2.left_bumper) {
                MovementMacros.ShootHighGoalAsync();
                //   wobbleManipulator2.offAngle();
            }*/
            if(!movement.pathFollowerIsActive()) {
                wobbleManipulator2.setposclose(buttonAswitch.isTriggered(gamepad1.a));
                wobbleManipulator2.upmediumdown(gamepad2.b, gamepad2.x); // correct
                conveyor.setConveyorPower(buttonBackswitch.isTriggered(gamepad2.left_trigger > 0.5) ? 1 : 0);
                shooter.setShootingMode(shooterOnOffSwitch.isTriggered(gamepad2.a) ?
                        shooterSpeedSwitch.isTriggered(gamepad2.y) ?
                                rpm.ShooterMode.POWERSHOT
                                : rpm.ShooterMode.HIGHGOAL
                        : rpm.ShooterMode.OFF);
                conveyor.setBackmust(gamepad2.right_trigger > 0.5);
             //   telemetry.addData("correct",shooter.isCorrectRpm());
                conveyor.OFFcolorlock(gamepad2.left_bumper);
                //   powerconveyor = (buttonspeedConveyor.isTriggered(gamepad2.right_bumper)?-1:1);
                if (gamepad1.x) {
                    ShootPOWERSHOTAngle();
                }
                if (gamepad1.b)
                    shooter.feedRing();
                if (threeRingPresser.isTriggered(gamepad1.right_stick_button))
                    shooter.feedRings();
            }
                double turn = 0;
                double y;
                double x;
                if (gamepad1.left_bumper) turn -= 0.25;
                else turn -= gamepad1.left_trigger;
                if (gamepad1.right_bumper) turn += 0.25;
                else turn += gamepad1.right_trigger;
                //  turn +=gamepad1.right_stick_x;
                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                if (gamepad1.dpad_up)
                    y += 1;
                if (gamepad1.dpad_down)
                    y = -1;
                if (gamepad1.dpad_left)
                    x = -1;
                if (gamepad1.dpad_right)
                    x += 1;

            movement.humanSetVelocity(x, y, turn);
            spinOnce();
        }
    }
}
