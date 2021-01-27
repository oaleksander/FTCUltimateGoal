package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import org.firstinspires.ftc.teamcode.robot.WobbleManipulator2;
import org.firstinspires.ftc.teamcode.robot.rpm;
import org.firstinspires.ftc.teamcode.superclasses.MotionTask;

import java.util.ArrayList;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.delay;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.getOpMode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.opMode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.spinOnce;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator2;

public class MovementMacros {

    static byte xSign = 1;
    static byte sideSign = 1;

    public MovementMacros(byte xSign, byte sideSign) {
        MovementMacros.xSign = xSign;
        MovementMacros.sideSign = sideSign;
    }

    private static Vector2D getHighGoalPose()
    {
        return new Vector2D(93.9174*xSign,182.691);
    }

    private static final double highGoalShootingDistance = 234.0;
    private static final double highGoalShootingAngle = toRadians(-5.3);

    private static Pose2D getHighGoalShootingPose()
    {
        Pose2D error = movement.getError(new Pose2D(getHighGoalPose(),Double.NaN));
        double angle = Range.clip(error.acot(),-11,11);
        return new Pose2D(getHighGoalPose().minus(new Vector2D(0,highGoalShootingDistance).rotatedCW(angle)),angle+highGoalShootingAngle);
    }
    public static void ShootHighGoal() {
        shooter.setShootingMode(rpm.ShooterMode.HIGHGOAL);
        movement.Pos(getHighGoalShootingPose());
        ElapsedTime shooterAccelerationTimeout = new ElapsedTime();
        while (WoENrobot.getOpMode().opModeIsActive() &&(( !shooter.isCorrectRpm(10) && shooterAccelerationTimeout.seconds()<3)))
            spinOnce();
        shooter.feedRings();
        delay(1050);
        shooter.setShootingMode(rpm.ShooterMode.OFF);
    }
    public static void ShootHighGoalAsync() {
        shooter.setShootingMode(rpm.ShooterMode.HIGHGOAL);
        movement.followPath(new MotionTask(getHighGoalShootingPose(), ()->{
            ElapsedTime shooterAccelerationTimeout = new ElapsedTime();
            while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds()<3 && movement.pathFollowerIsActive())
                spinOnce();
            shooter.feedRings();}));
      //  while(movement.pathFollowerIsActive()&&getOpMode().opModeIsActive()) {spinOnce();}
    }

    private static Vector2D getWobblePose()
    {
        switch(openCVNode.getStackSize())
        {
            case FOUR:
                return new Vector2D(xSign * 150.3809, 150.2761);
            case ONE:
                return new Vector2D(xSign * 89.9835, 90.3346);
            case ZERO:
            default:
                return new Vector2D(xSign * 150.3809, 30.1596);
        }
    }

    private static Vector2D getRingStackPose()
    {
        return new Vector2D(90.3747*xSign-2,-56.9019);
//    return new Vector2D(90.3747*xSign,-56.9019);
    }

    public static boolean PickupRings()
    {
        Pose2D error = movement.getError(new Pose2D(getRingStackPose(),Double.NaN));
        switch (openCVNode.getStackSize())
        {
            case FOUR:
                movement.Pos(new Pose2D(getRingStackPose().minus(new Vector2D(0,48).rotatedCW(error.acot())),error.acot()+Math.PI));
                conveyor.setConveyorPower(1);
                movement.Pos(new Pose2D(getRingStackPose().minus(new Vector2D(0,20-18).rotatedCW(error.acot())),error.acot()+Math.PI),0.3,1);
                delay(1000);
                ShootHighGoal();
                movement.Pos(new Pose2D(getRingStackPose().minus(new Vector2D(0,20-39).rotatedCW(error.acot())),error.acot()+Math.PI),0.6,1);
                delay(750);
                ShootHighGoal();
                //conveyor.setConveyorPower(0);
                break;
            case ONE:
                conveyor.setConveyorPower(1);
                movement.Pos(new Pose2D(getRingStackPose().minus(new Vector2D(0,20-30).rotatedCW(error.acot())),error.acot()+Math.PI),0.8,1);
                delay(1000);
                ShootHighGoal();
                //conveyor.setConveyorPower(0);
                break;
            case ZERO:
            default:
                return false;
        }
        return true;
    }

    private static final Vector2D wobblePlacementOffset = new Vector2D(11.8425,39.25);//new Vector2D(11.8425,33.25);

    public static void MoveWobble_experimental() {
        wobbleManipulator2.setposclose(true);
        wobbleManipulator2.changepos(WobbleManipulator2.positions.medium);
        Vector2D wobblePose = getWobblePose();
        Vector2D error = (Vector2D)movement.getError(new Pose2D(wobblePose,Double.NaN));
        movement.followPath(new MotionTask(wobblePose.minus(wobblePlacementOffset.rotatedCW(error.acot())),error.acot(), ()->{}));
        while(movement.pathFollowerIsActive()&&getOpMode().opModeIsActive()) {
            if (movement.getError(new Pose2D(wobblePose,Double.NaN)).radius()<75)
                wobbleManipulator2.changepos(WobbleManipulator2.positions.down);
            spinOnce();
        }
        wobbleManipulator2.changepos(WobbleManipulator2.positions.down);
        wobbleManipulator2.setposclose(false);
        WoENrobot.delay(300);
    }

    private static final double yParkLine = 26.462;
    private static final double robotYbackLength = 29.85498;
    private static final double robotYfrontLength = 37.2;
    private static final double parkingTolerance = 10;

    public static void Park() {
        if(odometry.getRobotCoordinates().y>yParkLine) {
            movement.Pos(new Pose2D(89.6372*xSign+67.3092*sideSign, yParkLine+robotYbackLength-parkingTolerance, 0));
        }
        else {
            wobbleManipulator2.changepos(WobbleManipulator2.positions.medium);
            movement.Pos(new Pose2D(89.6372*xSign+67.3092*sideSign, yParkLine-robotYfrontLength+parkingTolerance, 0));
            delay(50);
        }
    }
    public enum posPowerShots {
        LEFT,
        MEDIUM,
        RIGHT
    }
    private static Vector2D getPowerShotPos(MovementMacros.posPowerShots PosPowerShots)
    {
        switch (PosPowerShots) {
            case LEFT:
                return new Vector2D(9.600*xSign,182.691);
            case MEDIUM:
                return new Vector2D(28.595*xSign,182.691);
            case RIGHT:
            default:
                return new Vector2D(47.603*xSign,182.691);
        }
    }
    private static final double PowerShotShootingDistance = 200.4089;
    private static final double PowerShotShootingAngle = toRadians(-4.7);
    private static Vector2D getPowerShotShootingPose() {
    Pose2D error = movement.getError(new Pose2D(getPowerShotPos(posPowerShots.MEDIUM),Double.NaN));
        double angle = Range.clip(error.acot(),-11,11);
        return new Pose2D(getPowerShotPos(posPowerShots.MEDIUM).minus(new Vector2D(0,PowerShotShootingDistance).rotatedCW(angle)),angle+PowerShotShootingAngle);
    }
    public static void Shooting() {
        Shooting(true);
    }
    public static void Shooting(boolean NeedAngle) {
        if (sideSign*xSign == 1) {
            ShootHighGoal();
        }
        else if (NeedAngle){
            ShootPOWERSHOTAngle();
        }
        else
        {
            ShootPOWERSHOTPos();
        }
    }
    public static void ShootPOWERSHOTAngle() {  //rename
        movement.Pos(new Pose2D(Double.NaN, -20, Double.NaN));
        shooter.setShootingMode(rpm.ShooterMode.POWERSHOT);
        double pos = 31;
        double angle = 6.5;
        for (short i = 0; i < 3; i++){
           if (xSign == 1) {
               movement.Pos(new Pose2D(xSign * pos, -7.5, toRadians(angle)));
           }
           else {
               movement.Pos(new Pose2D(xSign * (pos + 5) , -7.5, toRadians(-angle)));
           }
           angle -= 5.4;
           delay(200);
           if(WoENrobot.getOpMode().gamepad1.x) {
               break;
           }
           //pos -= 18;
           // ElapsedTime shooterAccelerationTimeout = new ElapsedTime();
           // while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds()<3)
            //    spinOnce();
            shooter.feedRing();
            delay(200);
        }
        shooter.setShootingMode(rpm.ShooterMode.OFF);
    }


    public static void ShootPOWERSHOTPos() {  //rename
        movement.Pos(new Pose2D(Double.NaN, -15, Double.NaN));
        shooter.setShootingMode(rpm.ShooterMode.POWERSHOT);
        double pos = 50;
        double angle = 5.5;
        for (short i = 0; i < 3; i++){
            if (xSign == 1) {
                movement.Pos(new Pose2D(xSign * pos, -5, toRadians(angle)));
            }
            else {
                movement.Pos(new Pose2D(xSign * pos, -5, toRadians(3)));
            }
            //angle -= 6.4;
            if(WoENrobot.getOpMode().gamepad1.x) {
                break;
            }
            pos -= 18;
            // ElapsedTime shooterAccelerationTimeout = new ElapsedTime();
            // while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds()<3)
            //    spinOnce();
            shooter.feedRing();
            delay(200);
        }
        shooter.setShootingMode(rpm.ShooterMode.OFF);
    }

    @Deprecated
    public static void ShootTargets() {
        //shooter.setShootersetings(3850, 500);
        shooter.setShootingMode(rpm.ShooterMode.HIGHGOAL);
        if (sideSign == 1 && xSign == 1)
            movement.Pos(new Pose2D(xSign * 121, -48.5, toRadians(-8.5)));
        else if (sideSign == -1 && xSign == 1)
            movement.Pos(new Pose2D(xSign * 53, -30, toRadians(3)));
        else if (sideSign == -1 && xSign == -1)
            movement.Pos(new Pose2D(xSign * 147.5, -9.5, toRadians(10.5)));
        else // if (sideSign == 1 &&  xSign == -1)
            movement.Pos(new Pose2D(xSign * 61.5, -28, toRadians(-11.5)));
        ElapsedTime shooterAccelerationTimeout = new ElapsedTime();
        while (opMode.opModeIsActive() && !shooter.isCorrectRpm() && shooterAccelerationTimeout.seconds()<3)
            spinOnce();
        shooter.feedRings();
        delay(900);
        shooter.setShootingMode(rpm.ShooterMode.OFF);
    }

    @Deprecated
    public static void MoveWobble() {
        wobbleManipulator2.setposclose(true);
        wobbleManipulator2.changepos(WobbleManipulator2.positions.medium);
        if (sideSign * xSign == 1)
            switch (openCVNode.getStackSize()) {
                case FOUR:
                    movement.Pos(new Pose2D(xSign * 154, 113, toRadians(-8 * xSign)));
                    break;
                case ONE:
                    movement.Pos(new Pose2D(xSign * 144, 63, toRadians(-45 * xSign)));
                    break;
                case ZERO:
                default:
                    movement.Pos(new Pose2D(xSign * 145, -15, toRadians(0 * xSign)));
            }
        else

            switch (openCVNode.getStackSize()) {
                case FOUR:
                    movement.Pos(new Pose2D(xSign * 110, 116, toRadians(50 * xSign)));
                    break;
                case ONE:
                    movement.Pos(new Pose2D(xSign * 54.5 + 8.5, 63 + 3 * xSign, toRadians(50 * xSign)));
                    break;
                case ZERO:
                default:
                    movement.Pos(new Pose2D(xSign * 120, 0, toRadians(50 * xSign)));
            }
        wobbleManipulator2.changepos(WobbleManipulator2.positions.down);
        WoENrobot.delay(800);
        wobbleManipulator2.setposclose(false);
        WoENrobot.delay(600);
    }

    @Deprecated
    public static void PutRingsToLowGoal() {
        movement.Pos(new Pose2D((93.75 + 11.25 * xSign) * xSign, 150, toRadians(0)));
        wobbleManipulator2.setAngle(0.55);
        delay(1000);
        wobbleManipulator2.setAngle(0.5);
    }

}
