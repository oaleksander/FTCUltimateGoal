package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.WoENrobot;

@TeleOp(name="TeleOp COMPETITION", group="Competition")
public class Tele_competition extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.getInstance().initRobot(this);
        WoENrobot.getInstance().startRobot();
        while(opModeIsActive())
        {
            double turn = 0;
            if(gamepad2.left_bumper) turn -=0.25;
            else turn -= gamepad1.left_trigger;
            if(gamepad2.right_bumper) turn +=0.25;
            else turn += gamepad1.right_trigger;
            WoENrobot.drivetrain.holonomicMove(-gamepad1.left_stick_y, gamepad1.left_stick_x, turn);
        }
    }
}
