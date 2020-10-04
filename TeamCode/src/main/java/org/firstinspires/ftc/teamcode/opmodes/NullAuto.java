package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;

@Autonomous(name = "Nullauto (force init)")
public class NullAuto extends LinearOpMode {
    @Override
    public void runOpMode() {

        WoENrobot.getInstance().forceInitRobot(this);
        WoENrobot.getInstance().tFdetector.initialize();
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
        WoENrobot.getInstance().tFdetectorThread.start();
        WoENrobot.getInstance().startRobot();
        WoENrobot.getInstance().tFdetector.retrieveResult();
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
