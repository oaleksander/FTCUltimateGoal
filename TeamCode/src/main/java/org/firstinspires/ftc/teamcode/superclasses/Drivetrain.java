package org.firstinspires.ftc.teamcode.superclasses;

import org.firstinspires.ftc.teamcode.math.Vector3D;

public interface Drivetrain {
    void setRobotVelocity(double frontways, double sideways, double turn);

    default void setRobotVelocity(Vector3D move) {
        setRobotVelocity(move.y, move.x, move.z);
    }
}
