package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.CommandSender;
import org.firstinspires.ftc.teamcode.misc.motorAccelerationLimiter;
import org.firstinspires.ftc.teamcode.superclasses.Conveyor;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

import static java.lang.Math.abs;


public class Conveyor2 implements Conveyor {
    private LinearOpMode opMode;
    public static DcMotorEx conveyor = null;
    private final motorAccelerationLimiter conveyorPowerSender = new motorAccelerationLimiter(new CommandSender(p -> conveyor.setPower(-p))::send,6);
    private DistanceSensor sensorDistance;
    private final ElapsedTime motorCurrentTimer = new ElapsedTime();
    private final ElapsedTime stackDetectionTimer = new ElapsedTime();

    @Config
    static class ConveyorConfig {
        public static double motorLockingCurrentTimeout = 750;
        public static double motorLockingReverseTime = 750;
        public static double stackDetectionTimeout = 500;
        public static double stackDetectionReverseTime = 750;
        public static double distanceThreshold = -5.6;
        public static double currentThreshold = 3;
    }

    private boolean doAutomaticConveyorStopping = true;
    private boolean doReverseOnStop = true;

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void setAutomaticConveyorStopping(boolean doAutomaticConveyorStopping) {
        this.doAutomaticConveyorStopping = doAutomaticConveyorStopping;
    }


    public void setReverseAfterStop(boolean doReverseOnStop) {
        this.doReverseOnStop = doReverseOnStop;
    }


    public void setForceReverse(boolean forceReverse) {
        this.forceReverse = forceReverse;
    }

    private boolean forceReverse = false;
    private double currentMotorPower = 0;

    public void setConveyorPower(double requestedPower) {
        if(this.requestedPower!= requestedPower)
            motorCurrentTimer.reset();
        this.requestedPower = requestedPower;
    }

    private double requestedPower = 0;

    public void initialize() {
        initializecolor();
        initializedrive();
    }

    private void initializecolor() {
        sensorDistance = opMode.hardwareMap.get(DistanceSensor.class, "ringDetector");
    }

    private void initializedrive() {
        conveyor = opMode.hardwareMap.get(DcMotorEx.class, "odometerYL");
        conveyor.setDirection(DcMotorSimple.Direction.FORWARD); //!!! can break odometry
        conveyor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private static final double distanceQueryTimeout = 500;
    private double lastKnownDistance = 12;
    private final ElapsedTime distanceQueryTimer = new ElapsedTime();

    private double getdistance() {
        if (distanceQueryTimer.milliseconds() > distanceQueryTimeout) {
            lastKnownDistance = sensorDistance.getDistance(DistanceUnit.CM);
            distanceQueryTimer.reset();
        }
        return lastKnownDistance;
    }

    private static final double motorCurrentQueryTimeout = 500;
    private double lastKnownMotorCurrent = 0;
    private final ElapsedTime motorCurrentQueryTimer = new ElapsedTime();

    private double getAMPS() {
        if (motorCurrentQueryTimer.milliseconds() > motorCurrentQueryTimeout) {
            lastKnownMotorCurrent = abs(conveyor.getCurrent(CurrentUnit.AMPS));
            motorCurrentQueryTimer.reset();
        }
        return lastKnownMotorCurrent;
    }

    public void update() {
        if (!forceReverse) {
            if (doAutomaticConveyorStopping)
                if (!(doReverseOnStop && requestedPower == 0) && getdistance() >= ConveyorConfig.distanceThreshold) {
                    stackDetectionTimer.reset(); //Full collector detection
                }
            if ((stackDetectionTimer.milliseconds() > ConveyorConfig.stackDetectionTimeout || (doReverseOnStop && requestedPower == 0)) && doAutomaticConveyorStopping) { //reverse+stop in case of ring detection
                motorCurrentTimer.reset();
                currentMotorPower = stackDetectionTimer.milliseconds() < ConveyorConfig.stackDetectionTimeout + ConveyorConfig.stackDetectionReverseTime && doReverseOnStop ? -1 : 0;
            } else if (motorCurrentTimer.milliseconds() > ConveyorConfig.motorLockingCurrentTimeout) //reverse after locking
            {
                if (motorCurrentTimer.milliseconds() < ConveyorConfig.motorLockingReverseTime + ConveyorConfig.motorLockingCurrentTimeout) {
                    currentMotorPower = -requestedPower;
                } else
                    motorCurrentTimer.reset();
            } else {
                currentMotorPower = +requestedPower;
                if (getAMPS() < ConveyorConfig.currentThreshold)  //locking detection
                    motorCurrentTimer.reset();
            }
        } else {
            currentMotorPower = -1;
            motorCurrentTimer.reset();
        }
        conveyorPowerSender.setVelocity(currentMotorPower);

    }
}
