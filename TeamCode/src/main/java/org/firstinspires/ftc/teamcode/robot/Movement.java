package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain;
import org.firstinspires.ftc.teamcode.superclasses.Odometry;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Movement implements RobotModule {
    private static Odometry odometry;
    private static Drivetrain drivetrain;
    private LinearOpMode opMode = null;

    private static final double kP_distance = 0.008, kD_distance = 0.00034;
    private static final double kF_distance = 0.1;
    private static final double minError_distance = 5;
    private static final double kP_angle = 0.34, kD_angle = 0;
    private static final double kF_angle = 0.1;
    private static final double minError_angle = Math.toRadians(5);


    public Movement(Odometry Odometry, Drivetrain Drivetrain) {
        odometry = Odometry;
        drivetrain = Drivetrain;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initialize() {}


    public void Pos(Pose2D target) {


        Pose2D error = target.substract(odometry.getRobotCoordinates());
        Pose2D errold = error;
        double distanceError = error.radius();

        ElapsedTime looptime = new ElapsedTime();
        while (opMode.opModeIsActive() && (distanceError > minError_distance || abs(error.heading) > minError_angle) && looptime.seconds() < 4) {

            error = target.substract(odometry.getRobotCoordinates());

            Pose2D differr = new Pose2D(0,0,0);//error.substract(errold)).divideByDouble(looptime.seconds());
            looptime.reset();

            Vector3D control = new Vector3D(
                    error.x * kP_distance + differr.x * kD_distance,
                    error.y * kP_distance + differr.y * kD_distance,
                    error.heading * kP_angle + differr.heading * kP_angle);

            holonomicMoveFC(control);


            errold = error;
            distanceError = error.radius();
        }
        drivetrain.setRobotVelocity(0, 0, 0);
    }
    /*
        public boolean Pos(Pose2D target) {


        Pose2D error = target.substract(odometry.getRobotCoordinates());
        Pose2D errold = error;
        double distanceError = error.radius();

        ElapsedTime looptime = new ElapsedTime();
        if ((distanceError > minError_distance || abs(error.heading) > minError_angle) && looptime.seconds() < 4) {


            distanceError = error.radius();


            error = target.substract(odometry.getRobotCoordinates());
            if(distanceError>60)
            {
                error.heading=angleWrapHalf(error.acot());
            }

            Vector3D differr = new Vector3D(0,0,0);
            looptime.reset();

            Vector3D control = new Vector3D(
                    error.x * kP_distance + differr.x * kD_distance,
                    error.y * kP_distance + differr.y * kD_distance,
                    error.heading * kP_angle + differr.z * kP_angle);

            holonomicMoveFC(control);





            errold = error;
            return false;
        }
        else {
             return true;
        }
    }

     */

    public void holonomicMoveFC(Vector3D move) {
        Vector2D coordinates = new Vector2D(move.x, move.y).rotatedCW(-odometry.getRobotCoordinates().heading);
        drivetrain.setRobotVelocity(coordinates.y, coordinates.x, move.z);
    }

    public void holonomicMovePolar(double heading, double speed, double turn) {
        turn = Range.clip(turn, -1.0, 1.0);
        speed = Range.clip(speed, -1.0, 1.0);
        double frontways = speed * cos(heading);
        double sideways = speed * sin(heading);

        drivetrain.setRobotVelocity(frontways, sideways, turn);
    }

}
