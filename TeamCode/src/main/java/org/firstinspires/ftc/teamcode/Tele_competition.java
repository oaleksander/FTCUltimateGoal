package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop COMPETITION", group="")
public class Tele_competition extends WoENrobot {
    @Override
    public void runOpMode() {

        initRobot();
        startRobot();
        while(opModeIsActive())
        {
            double turn = 0;
            if(gamepad2.left_bumper) turn -=0.25;
            else turn -= gamepad1.left_trigger;
            if(gamepad2.right_bumper) turn +=0.25;
            else turn += gamepad1.right_trigger;
            omniMoveYX(-gamepad1.left_stick_y, gamepad1.left_stick_x, turn);
            grabBrick(gamepad1.x,gamepad1.y);
            grabFoundation(gamepad1.a);

        }
    }
}
