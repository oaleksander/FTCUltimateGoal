package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleConsumer;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.signum;

public class motorAccelerationLimiter {
    private final DoubleConsumer motorToControl;
    private final ElapsedTime looptime = new ElapsedTime();

    private double maxAcceleration;

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }

    private double currentVelocity = 0;

    public motorAccelerationLimiter(DoubleConsumer motorToControl, double maxAcceleration) {
        this.motorToControl = motorToControl;
        this.maxAcceleration = maxAcceleration;
        looptime.reset();
    }

    public void setVelocity(double requestedVelocity) {
        currentVelocity += min(abs(requestedVelocity - currentVelocity), abs(looptime.seconds() * maxAcceleration)) * signum(requestedVelocity - currentVelocity);
        if (requestedVelocity == 0)
            motorToControl.accept(0);
        else
            motorToControl.accept(currentVelocity);
        looptime.reset();
    }
}
