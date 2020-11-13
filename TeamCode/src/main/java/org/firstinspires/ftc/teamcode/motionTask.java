package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.math.Pose2D;

public class motionTask extends Pose2D {


    public ActionOnCompletion actionOnConpletion = () -> {};
    public motionTask(double x, double y, double heading, ActionOnCompletion actionOncompletion) {
        super(x,y,heading);
        this.actionOnConpletion = actionOncompletion;
    }

    public motionTask(double x, double y, double heading){
        super(x,y,heading);
    }

    public motionTask(double x, double y, ActionOnCompletion actionOncompletion) {
        super(x,y,Double.NaN);
        this.actionOnConpletion = actionOncompletion;
    }

    public motionTask(double x, double y) {
        super(x,y,Double.NaN);
    }


    public interface ActionOnCompletion
    {
        void execute();
    }
}
