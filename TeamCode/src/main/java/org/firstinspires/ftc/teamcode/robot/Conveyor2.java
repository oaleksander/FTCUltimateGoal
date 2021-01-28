package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.CommandSender;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

import static java.lang.Math.abs;

@Deprecated
public class Conveyor2 implements RobotModule {
    private LinearOpMode opMode;
    private DcMotorEx conveyor = null;
    private final CommandSender conveyorPowerSender = new CommandSender(p -> conveyor.setPower(-p));
    private DistanceSensor sensorDistance;
    private final ElapsedTime motorCurrentTimer = new ElapsedTime();
    private final ElapsedTime stackDetectionTimer = new ElapsedTime();
    private static final double motorLockingCurrentTimeout = 500;
    private static final double motorLockingReverseTime = 1000;
    private static final double stackDetectionTimeout = 500;
    private static final double stackDetectionReverseTime = 1000;
    private boolean doAutomaticConveyorStopping = true;
    private boolean doReverseOnStop = true;

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void setAutomaticConveyorStopping(boolean doAutomaticConveyorStopping) {
        this.doAutomaticConveyorStopping = doAutomaticConveyorStopping;
    }


    public void setReverseOnStop(boolean doReverseOnStop) {
        this.doReverseOnStop = doReverseOnStop;
    }


    public void setForceReverse(boolean forceReverse) {
        this.forceReverse = forceReverse;
    }

    private boolean forceReverse = false;
    private double currentMotorPower = 0;

    public void setConveyorPower(double requestedPower) {
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

    private static double distanceQueryTimeout = 500;
    private double lastKnownDistance = 12;
    private ElapsedTime distanceQueryTimer = new ElapsedTime();

    private double getdistance() {
        if (distanceQueryTimer.milliseconds() > distanceQueryTimeout) {
            lastKnownDistance = sensorDistance.getDistance(DistanceUnit.CM);
            distanceQueryTimer.reset();
        }
        return 10;
     //   return lastKnownDistance;
    }

    private static double motorCurrentQueryTimeout = 500;
    private double lastKnownMotorCurrent = 0;
    private ElapsedTime motorCurrentQueryTimer = new ElapsedTime();

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
                if (!(doReverseOnStop && requestedPower == 0) && getdistance() >= 2) {
                    stackDetectionTimer.reset(); //Full collector detection
                }
            if ((stackDetectionTimer.milliseconds() > stackDetectionTimeout || (doReverseOnStop && requestedPower == 0)) && doAutomaticConveyorStopping) { //reverse+stop in case of ring detection
                motorCurrentTimer.reset();
                if (stackDetectionTimer.milliseconds() < stackDetectionTimeout + stackDetectionReverseTime) {
                    currentMotorPower = -1;
                } else
                    currentMotorPower = 0;
            } else if (motorCurrentTimer.milliseconds() > motorLockingCurrentTimeout) //reverse after locking
            {
                if (motorCurrentTimer.milliseconds() < motorLockingReverseTime + motorLockingCurrentTimeout) {
                    currentMotorPower = -requestedPower;
                } else
                    motorCurrentTimer.reset();
            } else {
                currentMotorPower = +requestedPower;
                if (getAMPS() < 4) { //locking detection
                    motorCurrentTimer.reset();
                }
            }
        } else {
            currentMotorPower = -1;
            motorCurrentTimer.reset();
        }
        conveyorPowerSender.send(currentMotorPower);
    }
}
