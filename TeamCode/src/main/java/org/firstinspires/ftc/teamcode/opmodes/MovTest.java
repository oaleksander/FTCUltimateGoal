package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.WoENrobot;

@Autonomous(name = "Movement test")
public class MovTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.forceInitRobot(this);
        //  WoENrobot.tFdetector.initialize();
        //AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
        //   WoENrobot.tFdetectorThread.start();
        WoENrobot.startRobot();

        /*------------------------------------------------------------------------------------------------*/
    /*    switch(WoENrobot.tFdetector.retrieveResult())
        {
            case 4:
                WoENrobot.drivetrain.Pos(new Pose2D(50,100,Math.toRadians(45)));
                break;
            case 1:
                WoENrobot.drivetrain.Pos(new Pose2D(-50,80,Math.toRadians(-45)));
                break;
            default:
                WoENrobot.drivetrain.Pos(new Pose2D(0,1,Math.toRadians(0)));
                break;
        } */

        /*------------------------------------------------------------------------------------------------*/
        while (opModeIsActive()) {
            /*------------------------------------------------------------------------------------------------*/
// FINISH LOOP CODE GOES HERE
            /*------------------------------------------------------------------------------------------------*/
            telemetry.addData("Status", "Program finished (" + getRuntime() + ")");
            telemetry.update();
        }
    }
}
