package org.firstinspires.ftc.teamcode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.OpenCVNode;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
@TeleOp
public class callopencv extends LinearOpMode {
    OpenCVNode openCVNode = new OpenCVNode();
    @Override
    public void runOpMode(){
        WoENrobot.forceInitRobot(this);
        openCVNode.initialize();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("mean", openCVNode.getMean());
            telemetry.addData("getAspectRatio", openCVNode.getAspectRatio());
            telemetry.addData("getStackSize", openCVNode.getStackSize());
            telemetry.update();
            if (gamepad1.a){
                openCVNode.stopcam();
            }
        }
    }
}
