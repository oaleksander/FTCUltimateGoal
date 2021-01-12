package org.firstinspires.ftc.teamcode.superclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public interface RobotModule {

    void setOpMode(LinearOpMode opMode); //{this.opMode = opMode;}

    void initialize();

    default void initialize(LinearOpMode opMode) {
        setOpMode(opMode);
        initialize();
        reset();
    }

    default void reset() {
    }

    default void update() {
    }
}
