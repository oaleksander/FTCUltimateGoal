package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Nullauto (force init)")
public class NullAuto extends WoENrobot {
    @Override
    public void runOpMode() {

        forceInitRobot();
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
        startRobot();
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
        while(opModeIsActive())
        {
            telemetry.addData("Status", "Program finished ("+getRuntime()+")");
            telemetry.update();
        }
    }
}
