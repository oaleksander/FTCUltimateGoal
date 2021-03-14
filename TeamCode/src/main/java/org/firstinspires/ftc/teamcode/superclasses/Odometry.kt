package org.firstinspires.ftc.teamcode.superclasses

import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.math.Vector3D

interface Odometry {
    var robotCoordinates: Pose2D
    val robotVelocity: Vector3D
}