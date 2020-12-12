package org.firstinspires.ftc.teamcode.superclasses;

import org.firstinspires.ftc.teamcode.math.Pose2D;

public class MotionTask extends Pose2D {


    public ActionOnCompletion actionOnConpletion = () -> {};
    public MotionTask(double x, double y, double heading, ActionOnCompletion actionOncompletion) {
        super(x,y,heading);
        this.actionOnConpletion = actionOncompletion;
    }

    public MotionTask(double x, double y, double heading){
        super(x,y,heading);
    }

    public MotionTask(double x, double y, ActionOnCompletion actionOncompletion) {
        super(x,y,Double.NaN);
        this.actionOnConpletion = actionOncompletion;
    }

    public MotionTask(double x, double y) {
        super(x,y,Double.NaN);
    }


    public interface ActionOnCompletion
    {
        void execute();
    }
}
