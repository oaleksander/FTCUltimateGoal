package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain;
import org.firstinspires.ftc.teamcode.superclasses.MotionTask;
import org.firstinspires.ftc.teamcode.superclasses.Odometry;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

import java.util.ArrayList;
import java.util.Arrays;

import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;

public class Movement implements RobotModule {
    // private static final double kP_distance = 0.010, kD_distance = 0.00034;
    private static final double kP_distance = 3.1, kD_distance = 0;
    private double maxLinearVelocityFraction = 1;
    private double minLinearVelocityFraction = 0.125;
    private static final double minError_distance = 1;
    // private static final double kP_angle = 0.40, kD_angle = 0;
    private static final double kP_angle = 2.7, kD_angle = 0;
    private double maxAngleVelocityFraction = 1;
    private double minAngleVelocityFraction = 0.125;
    private static final double minError_angle = Math.toRadians(0.32);
    private static Odometry odometry;
    private static Drivetrain drivetrain;
    int nTargetPoint = 1;
    ArrayList<MotionTask> pathToFollow = new ArrayList<>();
    boolean bPathFollowerEnabled = false;
    boolean bPathFollowingFinished = false;
    boolean doActiveBraking = false;
    Pose2D previousTarget = new Pose2D();
    Pose2D previousError = new Pose2D();
    Vector3D requestedVelocityPercent = new Vector3D(0, 0, 0);
    private LinearOpMode opMode = null;
    private Thread actionOnCompletionExecutor = new Thread();
    private final ElapsedTime pathFollowingTimer = new ElapsedTime();
    public Movement(Odometry Odometry, Drivetrain Drivetrain) {
        odometry = Odometry;
        drivetrain = Drivetrain;
    }

    /**
     * Replaces NaNs in Pose2D with odometry coordinates
     *
     * @param pose2D Pose to remove NaNs from
     * @return Pose without NaNs
     */
    public static Pose2D removeNaN(Pose2D pose2D) {
        pose2D = pose2D.clone();
        if (Double.isNaN(pose2D.x)) pose2D.x = odometry.getRobotCoordinates().x;
        if (Double.isNaN(pose2D.y)) pose2D.y = odometry.getRobotCoordinates().y;
        if (Double.isNaN(pose2D.heading)) pose2D.heading = odometry.getRobotCoordinates().heading;
        return pose2D;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initialize() {
        reset();
    }

    public void setActiveBraking(boolean doActiveBraking) {
        this.doActiveBraking = doActiveBraking;
    }

    public void reset() {
        requestedVelocityPercent = new Vector3D(0, 0, 0);
        nTargetPoint = 1;
        bPathFollowerEnabled = false;
        bPathFollowingFinished = false;
        doActiveBraking = false;
    }

    public void update() {
        if (opMode.gamepad1.y) stopPathFollowing();
        bPathFollowingFinished = nTargetPoint >= pathToFollow.size();
        if (pathFollowerIsActive() && requestedVelocityPercent.radius() < 0.01) {
            if (pathFollowingTimer.seconds() > 5)
                nTargetPoint++;
            else {
                Pose2D currentTarget = removeNaN(pathToFollow.get(nTargetPoint));
                Pose2D previousTarget = removeNaN(pathToFollow.get(nTargetPoint - 1));
                if (movement.movePurePursuit(previousTarget, currentTarget, 45.72 * 1.5)) {
                    if (movement.moveLinear(currentTarget)) {
                        pathFollowingTimer.reset();
                        if (actionOnCompletionExecutor.getState() == Thread.State.NEW)
                            actionOnCompletionExecutor.start();
                        else if (actionOnCompletionExecutor.getState() == Thread.State.TERMINATED) {
                            nTargetPoint++;
                            if (nTargetPoint < pathToFollow.size())
                                actionOnCompletionExecutor = new Thread(pathToFollow.get(nTargetPoint).actionOnConpletion);
                        }
                    }
                }
            }
        } else if (requestedVelocityPercent.radius() > 0.01) {
            if(pathFollowerIsActive()) stopPathFollowing();
            drivetrain.setRobotVelocity(requestedVelocityPercent.multiply(drivetrain.getMaxVelocity()));
        }
        else
            if(pathToFollow.size()>0 && doActiveBraking)
                moveLinear(pathToFollow.get(pathToFollow.size()-1));
            else
            drivetrain.setRobotVelocity(0, 0, 0);
    }

    /**
     * Checks whether path follower was disabled or have finished its job
     *
     * @return Whether path follower is active
     */

    public boolean pathFollowerIsActive() {
        return bPathFollowerEnabled && !bPathFollowingFinished;
    }

    /**
     * Legacy synchronous go-to-point (for compatibility)
     *
     * @param target Point to approach
     */

    public void Pos(Pose2D target) {
        Pos(target,1,1);
    }

    /**
     * Legacy synchronous go-to-point (with custom speeds)
     *
     * @param target Point to approach
     * @param linearVelocityFraction Array of points (motion tasks)
     * @param angularVelocityFraction Array of points (motion tasks)
     */

    public void Pos(Pose2D target, double linearVelocityFraction, double angularVelocityFraction) {
        followPath(new MotionTask(target),linearVelocityFraction,angularVelocityFraction);
        while (pathFollowerIsActive() && opMode.opModeIsActive()) {
            Thread.yield();
        }
    }

    /**
     * Give path follower a task to go to point (with custom speeds)
     *
     * @param motionTask Points (motion tasks)
     */

    public void followPath(MotionTask motionTask)
    {
        followPath(motionTask, 1, 1);
    }

    /**
     * Give path follower a task to go to point
     *
     * @param motionTask Points (motion tasks)
     * @param linearVelocityFraction Array of points (motion tasks)
     * @param angularVelocityFraction Array of points (motion tasks)
     */

    public void followPath(MotionTask motionTask, double linearVelocityFraction, double angularVelocityFraction) {
        followPath(new ArrayList<>(Arrays.asList(motionTask)), linearVelocityFraction, angularVelocityFraction);
    }

    /**
     * Give path follower a task to follow an array of points
     *
     * @param pathToFollow Array of points (motion tasks)
     */

    public void followPath(ArrayList<MotionTask> pathToFollow)
    {
        followPath(pathToFollow, 1, 1);
    }

    /**
     * Give path follower a task to follow an array of points (with custom speeds)
     *
     * @param pathToFollow Array of points (motion tasks)
     * @param linearVelocityFraction Array of points (motion tasks)
     * @param angularVelocityFraction Array of points (motion tasks)
     */
    public void followPath(ArrayList<MotionTask> pathToFollow, double linearVelocityFraction, double angularVelocityFraction) {
        setMaxLinearVelocityFraction(linearVelocityFraction);
        setMaxAngleVelocityFraction(angularVelocityFraction);
        bPathFollowerEnabled = false;
        Thread.yield();
        this.pathToFollow = pathToFollow;
        pathToFollow.add(0, new MotionTask(odometry.getRobotCoordinates()));
        Thread.yield();
        nTargetPoint = 1;
        bPathFollowerEnabled = true;
        bPathFollowingFinished = false;
        actionOnCompletionExecutor = new Thread(pathToFollow.get(nTargetPoint).actionOnConpletion);
        pathFollowingTimer.reset();
    }

    /**
     * Disable path follower (and start receiving teleop instructions)
     */

    public void stopPathFollowing() {
        bPathFollowerEnabled = false;
    }

    public void setMaxLinearVelocityFraction(double maxLinearVelocityFraction) {
        this.maxLinearVelocityFraction = maxLinearVelocityFraction;
    }

    public void setMinLinearVelocityFraction(double minLinearVelocityFraction) {
        this.minLinearVelocityFraction = minLinearVelocityFraction;
    }

    public void setMaxAngleVelocityFraction(double maxAngleVelocityFraction) {
        this.maxAngleVelocityFraction = maxAngleVelocityFraction;
    }

    public void setMinAngleVelocityFraction(double minAngleVelocityFraction) {
        this.minAngleVelocityFraction = minAngleVelocityFraction;
    }

    /**
     * Set drivetrain speeds to approach given point
     *
     * @param targetPose      Target point (robot-centric)
     * @param linearVelocity  Robot linear velocity
     * @param angularVelocity Robot angular velocity
     */

    public void approachPosition(Pose2D targetPose, double linearVelocity, double angularVelocity) {

        linearVelocity = Range.clip(abs(linearVelocity),
                abs(linearVelocity)>kP_distance*minError_distance/2?drivetrain.getMaxVelocity().y*minLinearVelocityFraction:0,
                drivetrain.getMaxVelocity().y*maxLinearVelocityFraction);
        angularVelocity = Range.clip(abs(angularVelocity),
                abs(angularVelocity)>kP_angle*minError_angle/2?drivetrain.getMaxVelocity().z*minAngleVelocityFraction:0,
                drivetrain.getMaxVelocity().z*maxAngleVelocityFraction);

        Vector2D movementControl = new Vector2D(
                targetPose.x,
                targetPose.y).normalize().multiply(linearVelocity);

        Vector3D control = new Vector3D(movementControl,
                angularVelocity * signum(targetPose.heading));

        holonomicMoveFC(control);
    }

    /**
     * Move to point Linearly
     *
     * @param target Pose Target point
     * @return Whether robot is within minimum error tolerance
     */

    public boolean moveLinear(Pose2D target) {

        Pose2D error = getError(target);

        Vector3D diffError = new Vector3D(0, 0, 0);
        if (target.equals(previousTarget)) {
            Pose2D deltaError = error.substract(previousError);

            Vector3D currentVelocity = odometry.getRobotVelocity();
            diffError = new Vector3D(currentVelocity.x * signum(deltaError.x),
                    currentVelocity.y * signum(deltaError.y),
                    currentVelocity.z * signum(deltaError.heading));
        }

        previousError = error;
        previousTarget = target;
        approachPosition(error, error.radius() * kP_distance + ((Vector2D) diffError).radius() * kD_distance,
                error.heading * kP_angle + diffError.z * kD_angle);
        return !(abs(error.heading) >= minError_angle) && !(error.radius() >= minError_distance);
        //drivetrain.setRobotVelocity(0, 0, 0);
    }

    /**
     * Calculates positional error
     *
     * @param target Pose we want to be in
     * @return Difference between current position and given target
     */
    public Pose2D getError(Pose2D target) {
        target = removeNaN(target);
        return target.substract(odometry.getRobotCoordinates());
    }

/*
    public void Pos(Pose2D target) {


        Pose2D error = target.substract(odometry.getRobotCoordinates());
        Pose2D errold;
        double distanceError = error.radius();

        ElapsedTime movementTime = new ElapsedTime();
        while (opMode.opModeIsActive() && (distanceError >= minError_distance || abs(error.heading) >= minError_angle) && movementTime.seconds() < 4 &&!opMode.gamepad1.y) {

            errold = error;
            error = target.substract(odometry.getRobotCoordinates());

            distanceError = error.radius();
            if (distanceError > 100) {
                error.heading = angleWrapHalf(error.acot());
            }

            Pose2D deltaError = error.substract(errold);

            Vector3D currentVelocity = odometry.getRobotVelocity();
            Vector3D diffError = new Vector3D(currentVelocity.x * signum(deltaError.x),
                    currentVelocity.y * signum(deltaError.y),
                    currentVelocity.z * signum(deltaError.heading));

            Vector2D movementControl = new Vector2D(
                    error.x * kP_distance + diffError.x * kD_distance,
                    error.y * kP_distance + diffError.y * kD_distance);
          //  if (movementControl.radius() > drivetrain)
          //      movementControl.normalize();

            Vector3D control = new Vector3D(movementControl,
                    error.heading * kP_angle + diffError.z * kD_angle);

            holonomicMoveFC(control);


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

    /**
     * Move to point using Pure Pursuit
     *
     * @param originPoint     Trajectory starting point
     * @param targetPoint     Trajectory ending point
     * @param lookaheadRadius Pure pursuit lookahead radius
     * @return If robotPosition is within lookahead radius of the target
     */
    public boolean movePurePursuit(Vector2D originPoint, Vector2D targetPoint, double lookaheadRadius) {
        //   ComputerDebugging.sendLine(new FloatPoint(originPoint.x + 356.0 / 2, originPoint.y + 356.0 / 2), new FloatPoint(targetPoint.x + 356.0 / 2, targetPoint.y + 356.0 / 2));

        Pose2D robotPosition = odometry.getRobotCoordinates();
        if (originPoint.x == targetPoint.x && originPoint.y == targetPoint.y)
            return true;
        double a = originPoint.y - targetPoint.y;
        double b = targetPoint.x - originPoint.x;
        double c = originPoint.x * targetPoint.y - originPoint.y * targetPoint.x;

        double angle = targetPoint.minus(originPoint).acot();

        Vector2D lookAheadPoint = new Vector2D(
                (b * (b * robotPosition.x - a * robotPosition.y) - a * c) / (a * a + b * b),
                (a * (-b * robotPosition.x + a * robotPosition.y) - b * c) / (a * a + b * b)
        ).add(new Vector2D(0, lookaheadRadius).rotatedCW(angle));

        if (abs(angleWrap(angle - robotPosition.heading)) > Math.PI / 2) {
            angle += Math.PI;
        }

        //  ComputerDebugging.sendKeyPoint(new FloatPoint(lookAheadPoint.x + 356.0 / 2, lookAheadPoint.y + 356.0 / 2));

        Pose2D error = getError(new Pose2D(lookAheadPoint, lookAheadPoint.minus(robotPosition).acot()));
        if (abs(angleWrap(error.heading)) > Math.PI / 2) {
            error.heading = angleWrap(error.heading + Math.PI);
        }
        if (targetPoint.minus(originPoint).radius() <= lookAheadPoint.minus(originPoint).radius()) {
            error = getError(new Pose2D(targetPoint, angle));
            if (error.radius() < lookaheadRadius)
                return true;
            approachPosition(error, error.radius() * kP_distance, error.heading * kP_angle);
        } else {
            approachPosition(error, drivetrain.getMaxVelocity().y, error.heading * kP_angle);
        }
        return false;
    }

    /**
     * Receive teleop movement instructions
     *
     * @param x    x velocity (-1.0; 1.0)
     * @param y    x velocity (-1.0; 1.0)
     * @param turn angular velocity (-1.0; 1.0)
     */
    public void humanSetVelocity(double x, double y, double turn) {
        requestedVelocityPercent = new Vector3D(x, y, turn);
    }

    /**
     * Field-centric move
     *
     * @param move Robot velocities in cm/s
     */
    public void holonomicMoveFC(Vector3D move) {
        Vector2D coordinates = new Vector2D(move.x, move.y).rotatedCW(-odometry.getRobotCoordinates().heading);
        drivetrain.setRobotVelocity(coordinates.y, coordinates.x, move.z);
    }

   /* public void holonomicMovePolar(double heading, double speed, double turn) {
        turn = Range.clip(turn, -1.0, 1.0);
        speed = Range.clip(speed, -1.0, 1.0);
        double frontways = speed * cos(heading);
        double sideways = speed * sin(heading);

        drivetrain.setRobotVelocity(frontways, sideways, turn);
    }*/

}
