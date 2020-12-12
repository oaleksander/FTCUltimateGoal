package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.misc.ButtonSwitch;
import org.firstinspires.ftc.teamcode.misc.SinglePressButton;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator2;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;

@TeleOp(name = "TeleOp COMPETITION", group = "Competition")
public class Tele_competition extends LinearOpMode {
    @Override
    public void runOpMode() {
        initRobot(this);
        startRobot();
        ButtonSwitch buttonAswitch = new ButtonSwitch();
        ButtonSwitch buttonBackswitch = new ButtonSwitch();
        ButtonSwitch buttonspeedShooter = new ButtonSwitch();
      //  ButtonSwitch buttonspeedConveyor = new ButtonSwitch();

        ButtonSwitch buttonStartswitch = new ButtonSwitch();
        SinglePressButton threeRingPresser = new SinglePressButton();
        //conveyor.feeder.setPosition(0.06);
      //  double powerconveyor = 1;

        shooter.setShootersetings(3800,500);
        while (opModeIsActive()) {
            double turn = 0;
            double y = 0;
            double x = 0;
            if (gamepad1.left_bumper) turn -= 0.25;
            else turn -= gamepad1.left_trigger;
            if (gamepad1.right_bumper) turn += 0.25;
            else turn += gamepad1.right_trigger;
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
            wobbleManipulator2.setposclose(buttonAswitch.isTriggered(gamepad1.a));
            wobbleManipulator2.upmediumdown(gamepad2.b,gamepad2.x); // correct
            shooter.onshooter(buttonStartswitch.isTriggered(gamepad2.a));
            conveyor.setConveyorPower(buttonBackswitch.isTriggered(gamepad2.left_bumper)?1:0);
            shooter.setShootersetings((buttonspeedShooter.isTriggered(gamepad2.y)?3500:3800),500);
            conveyor.setBackmust(gamepad2.right_bumper);
         //   powerconveyor = (buttonspeedConveyor.isTriggered(gamepad2.right_bumper)?-1:1);

            if(gamepad1.b)
                conveyor.feedRing();
            if(threeRingPresser.isTriggered(gamepad1.right_stick_button))
                conveyor.feedrings();

            drivetrain.setRobotVelocity(y, x, turn);
      //      spinOnce();
        }
    }
}
