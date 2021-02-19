package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.StackSize
import org.firstinspires.ftc.teamcode.robot.WoENrobot.*
import org.firstinspires.ftc.teamcode.robot.rpm
import org.firstinspires.ftc.teamcode.superclasses.MotionTask
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator

object MovementMacros {
    var xSign: Byte = 1
    var sideSign: Byte = 1
    fun setSettings(xSign: Byte, sideSign: Byte) {
        MovementMacros.xSign = xSign
        MovementMacros.sideSign = sideSign
    }

    private val highGoalPose: Vector2D
        get() = Vector2D(93.9174 * xSign, 182.691)
    private const val highGoalShootingDistance = 234.0
    private val highGoalShootingAngle = Math.toRadians(-5.3)
    private val highGoalShootingPose: Pose2D
        get() {
            val error = movement.getError(Pose2D(highGoalPose, Double.NaN))
            val angle = Range.clip(error.acot(), Math.toRadians(-13.0), Math.toRadians(13.0))
            return Pose2D(highGoalPose-Vector2D(0.0, highGoalShootingDistance).rotatedCW(angle), angle + highGoalShootingAngle)
        }

    fun ShootHighGoal() {
        shooter.shootingMode = rpm.ShooterMode.HIGHGOAL
        movement.pos(highGoalShootingPose)
        val shooterAccelerationTimeout = ElapsedTime()
        while (getOpMode().opModeIsActive() && !shooter.isCorrectRpm(10.0) && shooterAccelerationTimeout.seconds() < 3)
            spinOnce()
        shooter.feedRings()
        delay(1050.0)
        shooter.shootingMode = rpm.ShooterMode.OFF
    }

    fun ShootHighGoalAsync() {
        shooter.shootingMode = rpm.ShooterMode.HIGHGOAL
        movement.followPath(MotionTask(highGoalShootingPose) {
            val shooterAccelerationTimeout = ElapsedTime()
            while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds() < 3 && movement.pathFollowerIsActive())
                spinOnce()
            if (movement.pathFollowerIsActive()) shooter.feedRings()
        })
        //  while(movement.pathFollowerIsActive()&&getOpMode().opModeIsActive()) {spinOnce();}
    }

    private val wobblePose: Vector2D
        get() = when (openCVNode.stackSize) {
            StackSize.FOUR -> Vector2D(xSign * 150.3809, 150.2761)
            StackSize.ONE -> Vector2D(xSign * 89.9835, 90.3346)
            else -> Vector2D(xSign * 150.3809, 30.1596)
        }
    private val partnerWobblePose: Vector2D = Vector2D(93.91741046 * xSign - 30.1416 * sideSign, -120.3139)

    //    return new Vector2D(90.3747*xSign,-56.9019);
    private val ringStackPose: Vector2D = Vector2D(90.3747 * xSign - 2, -56.9019)

    //    return new Vector2D(90.3747*xSign,-56.9019);
    fun PickupRings(): Boolean {
        val error = movement.getError(Pose2D(ringStackPose, Double.NaN))
        when (openCVNode.stackSize) {
            StackSize.FOUR -> {
                movement.pos(Pose2D(ringStackPose.minus(Vector2D(0.0, 48.0).rotatedCW(error.acot())), error.acot() + Math.PI), 1.0, 1.0, 5.0, Math.toRadians(5.0))
                conveyor.setConveyorPower(1.0)
                movement.pos(Pose2D(ringStackPose.minus(Vector2D(0.0, 20.0 - 18.0).rotatedCW(error.acot())), error.acot() + Math.PI), 0.3, 1.0, 3.0, Math.toRadians(3.0))
                delay(1000.0)
                conveyor.setConveyorPower(0.0)
                ShootHighGoal()
                conveyor.setConveyorPower(1.0)
                movement.pos(Pose2D(ringStackPose.minus(Vector2D(0.0, 20.0 - 39.0).rotatedCW(error.acot())), error.acot() + Math.PI), 0.6, 1.0, 3.0, Math.toRadians(3.0))
                delay(750.0)
                ShootHighGoal()
                conveyor.setConveyorPower(0.0)
            }
            StackSize.ONE -> {
                conveyor.setConveyorPower(1.0)
                movement.pos(Pose2D(ringStackPose.minus(Vector2D(0.0, 20.0 - 30.0).rotatedCW(error.acot())), error.acot() + Math.PI), 0.8, 1.0, 3.0, Math.toRadians(3.0))
                delay(1000.0)
                ShootHighGoal()
                conveyor.setConveyorPower(0.0)
            }
            StackSize.ZERO -> return false
            else -> return false
        }
        return true
    }

    private val wobblePlacementOffset = Vector2D(11.8425, 39.25) //new Vector2D(11.8425,33.25);
    fun MoveWobble() {
        wobbleManipulator.grabWobble(true)
        wobbleManipulator.setAngle(WobbleManipulator.Position.MEDIUM)
        val wobblePose = wobblePose
        val error: Vector2D = movement.getError(Pose2D(wobblePose, Double.NaN))
        movement.followPath(MotionTask(wobblePose.minus(Vector2D(0.0, wobblePlacementOffset.radius()).rotatedCW(error.acot())), error.acot() - wobblePlacementOffset.acot()), 1.0, 1.0, 2.0, Math.toRadians(2.0))
        while (movement.pathFollowerIsActive() && getOpMode().opModeIsActive()) {
            if (movement.getError(Pose2D(wobblePose, Double.NaN)).radius() < 75)
                wobbleManipulator.setAngle(WobbleManipulator.Position.DOWN)
            spinOnce()
        }
        wobbleManipulator.setAngle(WobbleManipulator.Position.DOWN)
        wobbleManipulator.grabWobble(false)
        delay(300.0)
    }

    fun PickSecondWobble() {
        wobbleManipulator.grabWobble(false)
        wobbleManipulator.setAngle(WobbleManipulator.Position.MEDIUM)
        val wobblePose = partnerWobblePose
        val error: Vector2D = movement.getError(Pose2D(wobblePose, Double.NaN))
        movement.pos(Pose2D(Double.NaN, Double.NaN, error.acot()), 1.0, 1.0, 1.0, Math.toRadians(75.0))
        movement.followPath(MotionTask(wobblePose.minus(Vector2D(0.0, wobblePlacementOffset.radius()).rotatedCW(error.acot())), error.acot() - wobblePlacementOffset.acot()), 1.0, 1.0, 1.5, Math.toRadians(1.0))
        while (movement.pathFollowerIsActive() && getOpMode().opModeIsActive()) {
            if (movement.getError(Pose2D(wobblePose, Double.NaN)).radius() < 75 )
                wobbleManipulator.setAngle(WobbleManipulator.Position.DOWN)
            spinOnce()
        }
        wobbleManipulator.setAngle(WobbleManipulator.Position.DOWN)
        delay(200.0)
        wobbleManipulator.grabWobble(true)
        delay(300.0)
    }

    private const val yParkLine = 26.462
    private const val robotYbackLength = 29.85498
    private const val robotYfrontLength = 37.2
    private const val parkingTolerance = 5.0
    fun Park() {
        if (odometry.robotCoordinates.y > yParkLine) {
            movement.pos(Pose2D(89.6372 * xSign + 67.3092 * sideSign, yParkLine + robotYbackLength - parkingTolerance, 0.0), 1.0, 1.0, parkingTolerance, Math.toRadians(5.0))
        } else {
            wobbleManipulator.setAngle(WobbleManipulator.Position.MEDIUM)
            movement.pos(Pose2D(89.6372 * xSign + 67.3092 * sideSign, yParkLine - robotYfrontLength + parkingTolerance, 0.0), 1.0, 1.0, parkingTolerance, Math.toRadians(5.0))
            delay(50.0)
        }
    }

    private fun getPowerShotPos(PosPowerShots: posPowerShots): Vector2D {
        return when (PosPowerShots) {
            posPowerShots.LEFT -> Vector2D(9.600 * xSign, 182.691)
            posPowerShots.MEDIUM -> Vector2D(28.595 * xSign, 182.691)
            posPowerShots.RIGHT -> Vector2D(47.603 * xSign, 182.691)
        }
    }

    private const val PowerShotShootingDistance = 200.4089
    private val PowerShotShootingAngle = Math.toRadians(-4.7)
    private val powerShotShootingPoses: Array<Pose2D>
        get() {
            val angle = Range.clip(movement.getError(Pose2D(getPowerShotPos(posPowerShots.MEDIUM), Double.NaN)).acot(), Math.toRadians(-17.0), Math.toRadians(17.0))
            val shootingCoordinates = getPowerShotPos(posPowerShots.MEDIUM).minus(Vector2D(0.0, PowerShotShootingDistance).rotatedCW(angle))
            val angleLeft = getPowerShotPos(posPowerShots.LEFT).minus(shootingCoordinates)
                .acot() - getPowerShotPos(posPowerShots.MEDIUM).minus(shootingCoordinates).acot()
            val angleRight = getPowerShotPos(posPowerShots.RIGHT).minus(shootingCoordinates)
                .acot() - getPowerShotPos(posPowerShots.MEDIUM).minus(shootingCoordinates).acot()
            return arrayOf(
                Pose2D(shootingCoordinates, angle + PowerShotShootingAngle + angleLeft),
                Pose2D(shootingCoordinates, angle + PowerShotShootingAngle),
                Pose2D(shootingCoordinates, angle + PowerShotShootingAngle + angleRight)
            )
        }

    @JvmOverloads
    fun Shooting(NeedAngle: Boolean = true) {
        if (sideSign * xSign == 1) {
            ShootHighGoal()
        } else if (NeedAngle) {
            ShootPOWERSHOTAngle()
        } else {
            ShootPOWERSHOTPos()
        }
    }

    fun ShootPOWERSHOTAngle() {  //rename
        shooter.shootingMode = rpm.ShooterMode.POWERSHOT
        val pos = 31.0
        var angle = 6.5
        for (i in 0..2) {
            if (xSign.toInt() == 1) { movement.pos(Pose2D(xSign * pos, -7.5, Math.toRadians(angle))) }
            else {
                movement.pos(Pose2D(xSign * (pos + 5), -7.5, Math.toRadians(-angle)))
            }
            angle -= 5.4
            delay(200.0)
            if (getOpMode().gamepad1.x) {
                break
            }
            //pos -= 18;
            // ElapsedTime shooterAccelerationTimeout = new ElapsedTime();
            // while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds()<3)
            //    spinOnce();
            shooter.feedRing()
            delay(200.0)
        }
        shooter.shootingMode = rpm.ShooterMode.OFF
    }

    fun ShootPOWERSHOTPos() {  //rename
        shooter.shootingMode = rpm.ShooterMode.POWERSHOT
        var pos = 50.0
        val angle = 5.5
        for (i in 0..2) {
            if (xSign.toInt() == 1) {
                movement.pos(Pose2D(xSign * pos, -5.0, Math.toRadians(angle)))
            } else {
                movement.pos(Pose2D(xSign * pos, -5.0, Math.toRadians(3.0)))
            }
            //angle -= 6.4;
            if (getOpMode().gamepad1.x) {
                break
            }
            pos -= 18.0
            // ElapsedTime shooterAccelerationTimeout = new ElapsedTime();
            // while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds()<3)
            //    spinOnce();
            shooter.feedRing()
            delay(200.0)
        }
        shooter.shootingMode = rpm.ShooterMode.OFF
    }

    fun ShootPowerShotAngle_experimental() {
        shooter.shootingMode = rpm.ShooterMode.POWERSHOT
        for (shootingPose in powerShotShootingPoses) {
            movement.pos(shootingPose)
            delay(200.0)
            val shooterAccelerationTimeout = ElapsedTime()
            shooterAccelerationTimeout.reset()
            while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds() < 2) spinOnce()
            shooter.feedRing()
            delay(200.0)
        }
        shooter.shootingMode = rpm.ShooterMode.OFF
    }

    @Deprecated("")
    fun ShootTargets() {
        //shooter.setShootersetings(3850, 500);
        shooter.shootingMode = rpm.ShooterMode.HIGHGOAL
        if (sideSign.toInt() == 1 && xSign.toInt() == 1)
            movement.pos(Pose2D((xSign * 121).toDouble(), -48.5, Math.toRadians(-8.5)))
        else if (sideSign.toInt() == -1 && xSign.toInt() == 1)
            movement.pos(Pose2D((xSign * 53).toDouble(), -30.0, Math.toRadians(3.0)))
        else if (sideSign.toInt() == -1 && xSign.toInt() == -1) movement.pos(Pose2D(xSign * 147.5, -9.5, Math.toRadians(10.5)))
        else  // if (sideSign == 1 &&  xSign == -1)
            movement.pos(Pose2D(xSign * 61.5, -28.0, Math.toRadians(-11.5)))
        val shooterAccelerationTimeout = ElapsedTime()
        while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds() < 3)
            spinOnce()
        shooter.feedRings()
        delay(900.0)
        shooter.shootingMode = rpm.ShooterMode.OFF
    }

    @Deprecated("")
    fun PutRingsToLowGoal() {
        movement.pos(Pose2D((93.75 + 11.25 * xSign) * xSign, 150.0, Math.toRadians(0.0)))
        wobbleManipulator.setAngle(WobbleManipulator.Position.MEDIUM)
        delay(1000.0)
        wobbleManipulator.setAngle(WobbleManipulator.Position.UP)
    }

    enum class posPowerShots {
        LEFT, MEDIUM, RIGHT
    }
}