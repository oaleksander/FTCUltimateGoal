package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Deprecated
@Disabled
public class AI {
    private final ElapsedTime AItime = new ElapsedTime();

    private final double interval = 1000;
    private final double maxError = 3;

    public void initialize() {
        AItime.reset();
    }

    public void update() {
        if (AItime.milliseconds() > interval) {
            AItime.reset();
            if (Math.abs(5) > 3) {


            }
        }
    }
}
