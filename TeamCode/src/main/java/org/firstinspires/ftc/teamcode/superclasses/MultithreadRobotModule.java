package org.firstinspires.ftc.teamcode.superclasses;

public abstract class MultithreadRobotModule extends RobotModule{

    @Deprecated
    @Override
    public final void update() {
       // updateControlHub();
       // updateExpansionHub();
       // updateOther();
    }

    public abstract void updateControlHub();

    public abstract void updateExpansionHub();

    public abstract void updateOther();
}
