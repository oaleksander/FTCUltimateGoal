package org.firstinspires.ftc.teamcode.superclasses;

public abstract class MultithreadRobotModule extends RobotModule{

    @Deprecated
    @Override
    public final void update() {
        updateControlHub();
        updateExpansionHub();
        updateOther();
    }

    public synchronized void updateControlHub(){}

    public synchronized void updateExpansionHub(){}

    public void updateOther(){
    }
}
