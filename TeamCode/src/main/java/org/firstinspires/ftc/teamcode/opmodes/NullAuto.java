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
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
        WoENrobot.getInstance().startRobot();
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
