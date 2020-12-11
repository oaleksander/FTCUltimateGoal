package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.ButtonSwitch;
import org.firstinspires.ftc.teamcode.misc.SinglePressButton;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.drivetrain;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.initRobot;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator2;
@TeleOp(name = "TeleOp COMPETITION single", group = "Competition")
public class Tele_competition_single  extends LinearOpMode {
    @Override
    public void runOpMode() {
        initRobot(this);
        startRobot();
        ButtonSwitch buttonAswitch = new ButtonSwitch();
        ButtonSwitch buttonBackswitch = new ButtonSwitch();

        ButtonSwitch buttonStartswitch = new ButtonSwitch();

        SinglePressButton threeRingPresser = new SinglePressButton();

        shooter.setShootersetings(5044,500);
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
            wobbleManipulator2.upmediumdown(gamepad1.y,gamepad1.x); // correct
            shooter.onshooter(buttonStartswitch.isTriggered(gamepad1.start));
            conveyor.setConveyorPower(buttonBackswitch.isTriggered(gamepad1.back)?1:0);
            if(gamepad1.b)
                conveyor.feedRing();
            if(threeRingPresser.isTriggered(gamepad1.right_stick_button))
                conveyor.feedrings();

            drivetrain.setRobotVelocity(y, x, turn);
            //      spinOnce();
        }
    }
}
