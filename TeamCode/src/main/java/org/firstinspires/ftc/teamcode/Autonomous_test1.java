package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_test 1")
public class Autonomous_test1 extends WoENrobot {
    @Override
    public void runOpMode() {

        forceInitRobot();
        AutoTransitioner.transitionOnStop(this, "Teleop COMPETITION");
        //startOpenCV();
        startRobot();
        //odometry.start();
/*------------------------------------------------------------------------------------------------*/
        sleep(1000);
        //Pos(20,-15,0);
/*------------------------------------------------------------------------------------------------*/
        while(opModeIsActive())
        {
            telemetry.addData("Status", "Program finished ("+getRuntime()+")");
            telemetry.update();
        }
    }
}
