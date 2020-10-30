package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

@Autonomous(name = "Do nothing auto (template)")
public class NullAuto extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.forceInitRobot(this);
     //   WoENrobot.tFdetector.initialize();
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
      //  WoENrobot.tFdetectorThread.start();
        WoENrobot.startRobot();
     //   WoENrobot.tFdetector.retrieveResult();
/*------------------------------------------------------------------------------------------------*/
// AUTO CODE GOES HERE
/*------------------------------------------------------------------------------------------------*/
        while(opModeIsActive())
        {
/*------------------------------------------------------------------------------------------------*/
// FINISH LOOP CODE GOES HERE
/*------------------------------------------------------------------------------------------------*/
            telemetry.addData("Status", "Program finished ("+getRuntime()+")");
            telemetry.update();
        }
    }
}
