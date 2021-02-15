package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.superclasses.Odometry
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain
import org.firstinspires.ftc.teamcode.superclasses.RobotModule
import org.firstinspires.ftc.teamcode.robot.Movement
import org.firstinspires.ftc.teamcode.superclasses.MotionTask
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.robot.WoENrobot
import org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig
import kotlin.jvm.JvmOverloads
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.MathUtil
import java.util.*
import kotlin.math.abs
import kotlin.math.sign

class Movement(private val odometry: Odometry, private val drivetrain: Drivetrain) : RobotModule {
    @Config
    internal object MovementConfig {
        @JvmField var lookaheadRadius = 45.72
        @JvmField var kP_distance = 3.9
        @JvmField var kD_distance = 0.15
        @JvmField var kI_distance = 0.6
        @JvmField var kP_angle = 3.6
        @JvmField var kD_angle = 0.15
        @JvmField var kI_angle = 0.6
        @JvmField var antiWindupFraction_distance = 0.17
        @JvmField var antiWindupFraction_angle = 0.17
    }

    private var maxLinearVelocityFraction = 1.0
    private var maxAngleVelocityFraction = 1.0
    private var minError_distance_current = minError_distance_default
    private var minError_angle_current = minError_angle_default
    var nTargetPoint = 1
    var pathToFollow = ArrayList<MotionTask>()
    var bPathFollowerEnabled = false
    var bPathFollowingFinished = false
    var doActiveBraking = false
    var previousTarget = Pose2D()
    var previousError = Pose2D()
    var integralError = Vector3D()
    var requestedVelocityPercent = Vector3D(0.0, 0.0, 0.0)
    private var opMode: LinearOpMode? = null
    private var actionOnCompletionExecutor = Thread()
    private val pathFollowingTimer = ElapsedTime()
    override fun setOpMode(opMode: LinearOpMode) {
        this.opMode = opMode
    }

    override fun initialize() {
        start()
    }

    fun setActiveBraking(doActiveBraking: Boolean) {
        this.doActiveBraking = doActiveBraking
    }

    override fun start() {
        requestedVelocityPercent = Vector3D(0.0, 0.0, 0.0)
        nTargetPoint = 1
        bPathFollowerEnabled = false
        bPathFollowingFinished = false
        doActiveBraking = false
        pathToFollow = ArrayList()
        pathToFollow.add(0, MotionTask(odometry.robotCoordinates))
    }

    // * 1.5
    override fun update() {
        if (opMode!!.gamepad1.y) stopPathFollowing()
        bPathFollowingFinished = nTargetPoint >= pathToFollow.size
        if (pathFollowerIsActive() && requestedVelocityPercent.radius() < 0.005) {
            if (pathFollowingTimer.seconds() > 4) nTargetPoint++ else {
                val currentTarget = removeNaN(pathToFollow[nTargetPoint], odometry.robotCoordinates)
                val previousTarget = removeNaN(pathToFollow[nTargetPoint - 1], odometry.robotCoordinates)
                if (WoENrobot.movement.movePurePursuit(previousTarget, currentTarget, MovementConfig.lookaheadRadius)) {
                    if (WoENrobot.movement.moveLinear(currentTarget)) {
                        pathFollowingTimer.reset()
                        if (actionOnCompletionExecutor.state == Thread.State.NEW) actionOnCompletionExecutor.start() else if (actionOnCompletionExecutor.state == Thread.State.TERMINATED) {
                            nTargetPoint++
                            if (nTargetPoint < pathToFollow.size) actionOnCompletionExecutor = Thread(pathToFollow[nTargetPoint].actionOnConpletion)
                        }
                    }
                }
            }
        } else if (requestedVelocityPercent.radius() > 0.005) {
            if (pathFollowerIsActive()) stopPathFollowing()
            drivetrain.setRobotVelocity(requestedVelocityPercent.multiply(drivetrain.maxVelocity))
        } else if (pathToFollow.size > 0 && doActiveBraking) moveLinear(pathToFollow[pathToFollow.size - 1]) else drivetrain.setRobotVelocity(0.0, 0.0, 0.0)
    }

    val currentTarget: Pose2D
        get() = removeNaN(pathToFollow[nTargetPoint], odometry.robotCoordinates)

    /**
     * Checks whether path follower was disabled or have finished its job
     *
     * @return Whether path follower is active
     */
    fun pathFollowerIsActive(): Boolean {
        return bPathFollowerEnabled && !bPathFollowingFinished
    }
    /**
     * Legacy synchronous go-to-point (with custom speeds)
     *
     * @param target                  Point to approach
     * @param linearVelocityFraction  Array of points (motion tasks)
     * @param angularVelocityFraction Array of points (motion tasks)
     */
    @JvmOverloads
    fun Pos(target: Pose2D?, linearVelocityFraction: Double = 1.0, angularVelocityFraction: Double = 1.0) {
        followPath(MotionTask(target), linearVelocityFraction, angularVelocityFraction, minError_distance_default, minError_angle_default)
        while (pathFollowerIsActive() && opMode!!.opModeIsActive()) {
            Thread.yield()
        }
    }

    /**
     * Legacy synchronous go-to-point (with custom speeds and tolerances)
     *
     * @param target                  Point to approach
     * @param linearVelocityFraction  Array of points (motion tasks)
     * @param angularVelocityFraction Array of points (motion tasks)
     * @param distanceTolerance       Minimum distance error
     * @param angularTolerance        Minimum angular error
     */
    fun Pos(target: Pose2D?, linearVelocityFraction: Double, angularVelocityFraction: Double, distanceTolerance: Double, angularTolerance: Double) {
        followPath(MotionTask(target), linearVelocityFraction, angularVelocityFraction, distanceTolerance, angularTolerance)
        while (pathFollowerIsActive() && opMode!!.opModeIsActive()) {
            Thread.yield()
        }
    }
    /**
     * Give path follower a task to go to point (with custom speeds and tolerances)
     *
     * @param motionTask              Points (motion tasks)
     * @param linearVelocityFraction  Array of points (motion tasks)
     * @param angularVelocityFraction Array of points (motion tasks)
     * @param distanceTolerance       Minimum distance error
     * @param angularTolerance        Minimum angular error
     */
    @JvmOverloads
    fun followPath(motionTask: MotionTask?, linearVelocityFraction: Double = 1.0, angularVelocityFraction: Double = 1.0, distanceTolerance: Double = minError_distance_default, angularTolerance: Double = minError_angle_default) {
        followPath(ArrayList(listOf(motionTask)), linearVelocityFraction, angularVelocityFraction, distanceTolerance, angularTolerance)
    }
    /**
     * Give path follower a task to follow an array of points (with custom speeds)
     *
     * @param pathToFollow            Array of points (motion tasks)
     * @param linearVelocityFraction  Array of points (motion tasks)
     * @param angularVelocityFraction Array of points (motion tasks)
     * @param distanceTolerance       Minimum distance error
     * @param angularTolerance        Minimum angular error
     */
    @JvmOverloads
    fun followPath(pathToFollow: ArrayList<MotionTask>, linearVelocityFraction: Double = 1.0, angularVelocityFraction: Double = 1.0, distanceTolerance: Double = minError_distance_default, angularTolerance: Double = minError_angle_default) {
        minError_distance_current = distanceTolerance
        minError_angle_current = angularTolerance
        setMaxLinearVelocityFraction(linearVelocityFraction)
        setMaxAngleVelocityFraction(angularVelocityFraction)
        bPathFollowerEnabled = false
        Thread.yield()
        this.pathToFollow = pathToFollow
        pathToFollow.add(0, MotionTask(odometry.robotCoordinates))
        Thread.yield()
        nTargetPoint = 1
        bPathFollowerEnabled = true
        bPathFollowingFinished = false
        actionOnCompletionExecutor = Thread(pathToFollow[nTargetPoint].actionOnConpletion)
        pathFollowingTimer.reset()
    }

    /**
     * Disable path follower (and start receiving teleop instructions)
     */
    fun stopPathFollowing() {
        bPathFollowerEnabled = false
    }

    fun setMaxLinearVelocityFraction(maxLinearVelocityFraction: Double) {
        this.maxLinearVelocityFraction = maxLinearVelocityFraction
    }

    fun setMinLinearVelocityFraction(minLinearVelocityFraction: Double) {
        MovementConfig.antiWindupFraction_distance = minLinearVelocityFraction
    }

    fun setMaxAngleVelocityFraction(maxAngleVelocityFraction: Double) {
        this.maxAngleVelocityFraction = maxAngleVelocityFraction
    }

    fun setMinAngleVelocityFraction(minAngleVelocityFraction: Double) {
        MovementConfig.antiWindupFraction_angle = minAngleVelocityFraction
    }

    /**
     * Set drivetrain speeds to approach given point
     *
     * @param targetPose      Target point (robot-centric)
     * @param linearVelocity  Robot linear velocity
     * @param angularVelocity Robot angular velocity
     */
    fun approachPosition(targetPose: Pose2D, linearVelocity: Double, angularVelocity: Double) {
        var linearVelocity = linearVelocity
        var angularVelocity = angularVelocity
        linearVelocity = Range.clip(abs(linearVelocity), 0.0,
                drivetrain.maxVelocity.y * maxLinearVelocityFraction)
        angularVelocity = Range.clip(abs(angularVelocity), 0.0,
                drivetrain.maxVelocity.z * maxAngleVelocityFraction)
        val movementControl = Vector2D(
                targetPose.x,
                targetPose.y).normalize().multiply(linearVelocity)
        val control = Vector3D(movementControl,
                angularVelocity * Math.signum(targetPose.heading))
        holonomicMoveFC(control)
    }

    private val moveControllerTimer = ElapsedTime()

    /**
     * Move to point Linearly
     *
     * @param target Pose Target point
     * @return Whether robot is within minimum error tolerance
     */
    fun moveLinear(target: Pose2D): Boolean {
        val error = getError(target)
        var diffError = Vector3D()
        if (target == previousTarget) {
            val deltaError = error.substract(previousError)
            integralError = integralError.add(Vector3D((error.x + previousError.x) * 0.5, (error.y + previousError.y) * 0.5, MathUtil.angleAverage(error.heading, previousError.heading)).scale(moveControllerTimer.seconds()))
            integralError = Vector3D(Range.clip(Math.abs(integralError.x), 0.0, MovementConfig.antiWindupFraction_distance * drivetrain.maxVelocity.x) * Math.signum(integralError.x),
                    Range.clip(Math.abs(integralError.y), 0.0, MovementConfig.antiWindupFraction_distance * drivetrain.maxVelocity.y) * Math.signum(integralError.y),
                    Range.clip(Math.abs(integralError.z), 0.0, MovementConfig.antiWindupFraction_angle * drivetrain.maxVelocity.z) * Math.signum(integralError.z))
            diffError = odometry.robotVelocity.scale(-1.0)
        } else integralError = Vector3D()
        moveControllerTimer.reset()
        previousError = error
        previousTarget = target
        approachPosition(error, error.radius() * MovementConfig.kP_distance + (diffError as Vector2D).radius() * MovementConfig.kD_distance + (integralError as Vector2D).radius() * MovementConfig.kI_distance,
                error.heading * MovementConfig.kP_angle + diffError.z * MovementConfig.kD_angle + integralError.z * MovementConfig.kI_angle)
        return Math.abs(error.heading) < minError_angle_current && error.radius() < minError_distance_current
        //drivetrain.setRobotVelocity(0, 0, 0);
    }

    /**
     * Calculates positional error
     *
     * @param target Pose we want to be in
     * @return Difference between current position and given target
     */
    fun getError(target: Pose2D): Pose2D {
        var target = target
        target = removeNaN(target, odometry.robotCoordinates)
        return target.substract(odometry.robotCoordinates)
    }

    /**
     * Move to point using Pure Pursuit
     *
     * @param originPoint     Trajectory starting point
     * @param targetPoint     Trajectory ending point
     * @param lookaheadRadius Pure pursuit lookahead radius
     * @return If robotPosition is within lookahead radius of the target
     */
    fun movePurePursuit(originPoint: Vector2D, targetPoint: Pose2D, lookaheadRadius: Double): Boolean {
        //   ComputerDebugging.sendLine(new FloatPoint(originPoint.x + 356.0 / 2, originPoint.y + 356.0 / 2), new FloatPoint(targetPoint.x + 356.0 / 2, targetPoint.y + 356.0 / 2));
        val robotPosition = odometry.robotCoordinates
        if (originPoint.x == targetPoint.x && originPoint.y == targetPoint.y) return true
        val a = originPoint.y - targetPoint.y
        val b = targetPoint.x - originPoint.x
        val c = originPoint.x * targetPoint.y - originPoint.y * targetPoint.x
        var angle = MathUtil.angleWrap(targetPoint.minus(originPoint).acot())
        val lookAheadPoint = Vector2D(
                (b * (b * robotPosition.x - a * robotPosition.y) - a * c) / (a * a + b * b),
                (a * (-b * robotPosition.x + a * robotPosition.y) - b * c) / (a * a + b * b)
        ).add(Vector2D(0.0, lookaheadRadius).rotatedCW(angle))
        if (abs(MathUtil.angleWrap(angle - if (originPoint.minus(robotPosition).radius() > lookaheadRadius) targetPoint.heading else robotPosition.heading)) > (if (targetPoint.minus(robotPosition).radius() > lookaheadRadius * 2) Math.PI / 2 else Math.PI / 4)) {
            angle = MathUtil.angleWrap(angle + if (targetPoint.minus(robotPosition).radius() > lookaheadRadius * 2 || Math.abs(MathUtil.angleWrap(targetPoint.heading - angle + Math.PI)) < Math.PI / 4) Math.PI else Math.PI / 2 * sign(MathUtil.angleWrap(targetPoint.heading - angle)))
        }

        //  ComputerDebugging.sendKeyPoint(new FloatPoint(lookAheadPoint.x + 356.0 / 2, lookAheadPoint.y + 356.0 / 2));

        //  Pose2D error = getError(new Pose2D(lookAheadPoint, lookAheadPoint.minus(robotPosition).acot()));
        var error = getError(Pose2D(lookAheadPoint, angle))
        // if (abs(angleWrap(error.heading)) > Math.PI / 2) {
        //     error.heading = angleWrap(error.heading + Math.PI);
        // }
        if (targetPoint.minus(originPoint).radius() <= lookAheadPoint.minus(originPoint).radius()) {
            error = getError(Pose2D(targetPoint, angle))
            if (error.radius() < lookaheadRadius) return true
            approachPosition(error, error.radius() * MovementConfig.kP_distance, error.heading * MovementConfig.kP_angle)
        } else {
            approachPosition(error, drivetrain.maxVelocity.y, error.heading * MovementConfig.kP_angle)
        }
        return false
    }

    /**
     * Receive teleop movement instructions
     *
     * @param x    x velocity (-1.0; 1.0)
     * @param y    x velocity (-1.0; 1.0)
     * @param turn angular velocity (-1.0; 1.0)
     */
    fun humanSetVelocity(x: Double, y: Double, turn: Double) {
        requestedVelocityPercent = Vector3D(x, y, turn)
    }

    /**
     * Field-centric move
     *
     * @param move Robot velocities in cm/s
     */
    fun holonomicMoveFC(move: Vector3D) {
        val coordinates = Vector2D(move.x, move.y).rotatedCW(-odometry.robotCoordinates.heading)
        drivetrain.setRobotVelocity(coordinates.y, coordinates.x, move.z)
    } /* public void holonomicMovePolar(double heading, double speed, double turn) {
        turn = Range.clip(turn, -1.0, 1.0);
        speed = Range.clip(speed, -1.0, 1.0);
        double frontways = speed * cos(heading);
        double sideways = speed * sin(heading);

        drivetrain.setRobotVelocity(frontways, sideways, turn);
    }*/

    companion object {
        private const val minError_distance_default = 1.0
        private val minError_angle_default = Math.toRadians(0.32)

        /**
         * Replaces NaNs in Pose2D with odometry coordinates
         *
         * @param pose2D Pose to remove NaNs from
         * @param odometryPose odometry Pose to replace NaNs with
         * @return Pose without NaNs
         */
        fun removeNaN(pose2D: Pose2D, odometryPose: Pose2D): Pose2D {
            var pose2D = pose2D
            pose2D = pose2D.clone()
            if (java.lang.Double.isNaN(pose2D.x)) pose2D.x = odometryPose.x
            if (java.lang.Double.isNaN(pose2D.y)) pose2D.y = odometryPose.y
            if (java.lang.Double.isNaN(pose2D.heading)) pose2D.heading = odometryPose.heading
            return pose2D
        }
    }
}