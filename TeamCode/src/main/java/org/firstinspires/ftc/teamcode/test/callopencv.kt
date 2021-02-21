package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.WoENrobot
import com.acmerobotics.dashboard.FtcDashboard

@TeleOp
@Disabled
class callopencv : LinearOpMode() {
    override fun runOpMode() {
        // WoENrobot.forceInitRobot(this);
        WoENrobot.openCVNode.initialize(this)
        waitForStart()
        FtcDashboard.getInstance().startCameraStream(WoENrobot.openCVNode.webcam, 0.0)
        while (opModeIsActive()) {
            telemetry.addData("mean", WoENrobot.openCVNode.mean)
            telemetry.addData("getAspectRatio", WoENrobot.openCVNode.aspectRatio)
            telemetry.addData("getStackSize", WoENrobot.openCVNode.stackSize)
            telemetry.update()
        }
        FtcDashboard.getInstance().stopCameraStream()
    }
}