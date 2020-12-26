package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

public class commandSender {
    private double timeout = 1000;
    private double lastValue = Double.NaN;
    private final ElapsedTime lastCommandTimer = new ElapsedTime();

    public commandSender()
    {
    }

    public commandSender(double timeout_ms)
    {
        timeout = timeout_ms;
    }
    public boolean shouldSendCommand(double value)
    {
        if(value!=lastValue || lastCommandTimer.milliseconds()>timeout)
        {
            lastValue = value;
            lastCommandTimer.reset();
            return true;
        }
        return false;
    }

}
