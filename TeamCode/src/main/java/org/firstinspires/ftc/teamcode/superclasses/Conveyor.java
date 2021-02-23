package org.firstinspires.ftc.teamcode.superclasses;

public interface Conveyor {

    void enableConveyor(boolean isEnabled);

    void setForceReverse(boolean forceReverse);

    void setReverseAfterStop(boolean doReverseOnStop);

    void setAutomaticConveyorStopping(boolean doAutomaticConveyorStopping);
}
