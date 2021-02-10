package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static org.firstinspires.ftc.teamcode.robot.Conveyor1.conveyorm;

@Deprecated
@Disabled
public class AI {
    private final ElapsedTime AItime = new ElapsedTime();

    private final double interval = 1000;
    private final double timeDiagnostic = 3000;
    private final double timeWait = 500;
    private final double maxError = 3;

    public void initialize() {
        AItime.reset();
    }

    public void update() {

    }

    public boolean diagnosticConveyor1(){
        AItime.reset();
        conveyorm.setPower(1);
        do {
            if (conveyorm.getCurrent(CurrentUnit.AMPS) > 0.5 && AItime.milliseconds() > timeWait) {
                return true;
            }
        }
        while (WoENrobot.opMode.opModeIsActive() && AItime.milliseconds() < timeDiagnostic);
        return false;
    }
}
