package org.firstinspires.ftc.teamcode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opencv.opencvnew;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
@TeleOp
public class callopencv extends LinearOpMode {
    opencvnew opencvnew = new opencvnew();
    @Override
    public void runOpMode(){
        WoENrobot.getInstance().forceInitRobot(this);
        opencvnew.initialize();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("mean", opencvnew.getMean());
            telemetry.addData("getAspectRatio", opencvnew.getAspectRatio());
            telemetry.addData("getStackSize", opencvnew.getStackSize());
            telemetry.update();
            if (gamepad1.a){
                opencvnew.stopcam();
            }
        }
    }
}
