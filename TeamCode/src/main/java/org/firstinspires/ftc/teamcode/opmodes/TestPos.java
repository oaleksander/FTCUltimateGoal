package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

@Autonomous(name = "Pos test")
public class TestPos extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.getInstance().forceInitRobot(this);
        WoENrobot.getInstance().tFdetector.initialize();
        //AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
        WoENrobot.getInstance().tFdetectorThread.start();
        WoENrobot.getInstance().startRobot();
        WoENrobot.getInstance().tFdetector.retrieveResult();
/*------------------------------------------------------------------------------------------------*/
// AUTO CODE GOES HERE
/*------------------------------------------------------------------------------------------------*/
        //WoENrobot.getInstance().drivetrain.Pos(new Pose2D(50,100,0));
        while(opModeIsActive())
        {

            WoENrobot.getInstance().drivetrain.holonomicMoveFC(new Pose2D(0.0,0.2,0));
/*------------------------------------------------------------------------------------------------*/
// FINISH LOOP CODE GOES HERE
/*------------------------------------------------------------------------------------------------*/
            telemetry.addData("Status", "Program finished ("+getRuntime()+")");
            telemetry.update();
        }

        WoENrobot.getInstance().drivetrain.holonomicMoveFC(new Pose2D(0,0,0));
    }
}
