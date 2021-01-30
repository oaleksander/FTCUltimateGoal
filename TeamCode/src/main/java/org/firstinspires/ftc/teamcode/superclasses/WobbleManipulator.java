package org.firstinspires.ftc.teamcode.superclasses;



public interface WobbleManipulator extends RobotModule{
    enum Position {UP, DOWN, MEDIUM}
    void setAngle(Position Positions);
    void grabWobble(boolean dograb);
    void upmediumdown(boolean upmedium, boolean updown);
}
