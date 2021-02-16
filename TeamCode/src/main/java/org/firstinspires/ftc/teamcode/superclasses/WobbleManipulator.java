package org.firstinspires.ftc.teamcode.superclasses;


public interface WobbleManipulator{
    enum Position {UP, DOWN, MEDIUM}

    void setAngle(Position Positions);

    void grabWobble(boolean dograb);

    void upmediumdown(boolean upmedium, boolean updown);
}
