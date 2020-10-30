package org.firstinspires.ftc.teamcode.superclasses;

import org.firstinspires.ftc.teamcode.math.Pose2D;

public interface Odometry extends RobotModule{
    void setRobotCoordinates(Pose2D coordinates);
    Pose2D getRobotCoordinates();

}
