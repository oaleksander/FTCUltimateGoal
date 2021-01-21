package org.firstinspires.ftc.teamcode.superclasses;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;

public interface Odometry extends RobotModule {
    Pose2D getRobotCoordinates();

    void setRobotCoordinates(Pose2D coordinates);

    Vector3D getRobotVelocity();


}
