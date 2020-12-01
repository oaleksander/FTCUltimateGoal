package org.firstinspires.ftc.teamcode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.WoENrobot;

@TeleOp
//@Disabled
public class callopencv extends LinearOpMode {

    @Override
    public void runOpMode() {
        new WoENrobot();
        WoENrobot.forceInitRobot(this);
        WoENrobot.openCVNode.initialize(this);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("mean", WoENrobot.openCVNode.getMean());
            telemetry.addData("getAspectRatio", WoENrobot.openCVNode.getAspectRatio());
            telemetry.addData("getStackSize", WoENrobot.openCVNode.getStackSize());
            telemetry.update();
        }
    }
}
