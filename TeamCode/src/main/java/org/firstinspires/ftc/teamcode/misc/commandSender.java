package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleConsumer;

public class commandSender{
    private double timeout = 1000;
    private double lastValue = Double.NaN;
    private final ElapsedTime lastCommandTimer = new ElapsedTime();
    DoubleConsumer doubleConsumer;

    public commandSender(DoubleConsumer doubleConsumer)
    {
        this.doubleConsumer = doubleConsumer;
    }

    public commandSender(DoubleConsumer doubleConsumer, double timeout_ms)
    {
        this.doubleConsumer = doubleConsumer;
        timeout = timeout_ms;
    }
    public void sendCommand(double value)
    {
        if(value!=lastValue || lastCommandTimer.milliseconds()>timeout)
        {
            doubleConsumer.accept(value);
            lastValue = value;
            lastCommandTimer.reset();
        }
    }

}
