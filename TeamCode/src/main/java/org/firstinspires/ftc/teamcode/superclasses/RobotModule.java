package org.firstinspires.ftc.teamcode.superclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class RobotModule {

    protected LinearOpMode opMode = null;

    public void setOpMode(LinearOpMode opMode)
    {
        this.opMode = opMode;
    }

    public void initialize() {}

    public final void initialize(LinearOpMode opMode) {
        setOpMode(opMode);
        initialize();
    }

    public void start() {}

    public void update() {}
}
