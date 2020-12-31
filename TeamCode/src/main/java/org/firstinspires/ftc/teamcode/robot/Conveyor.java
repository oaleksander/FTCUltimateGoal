package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;


public class Conveyor implements RobotModule {
    private LinearOpMode opMode;

    private DcMotorEx conveyorm = null;

    private Servo feeder = null;

    private DistanceSensor sensorDistance;

    private final ElapsedTime conveyorTime = new ElapsedTime();
    private final ElapsedTime backOnTime = new ElapsedTime();
    private final ElapsedTime pauseTime = new ElapsedTime();
    private final ElapsedTime feederTime = new ElapsedTime();


    private boolean full = false;
    private boolean backOn = false, stop = false;
    private boolean backMust = false;

    private byte i = 0;

    private double timelock = 0;
    private double conveyorPower = 0;

    private double distance = 0;

    private final double time = 125;
    private final double feederClose = 0.06;
    private final double feederOpen = 0.3;


    public void setOpMode(LinearOpMode OpMode) {
        opMode = OpMode;
    }


    public void initialize() {
        feederTime.reset();
        initializecolor();
        initializedrive();
        initializedservo();
    }

    public void reset() {
        feeder.setPosition(feederClose);
        conveyorm.setPower(0);
        backOn = false;
        stop = false;
        conveyorPower = 0;
        i=0;
    }

    private void initializecolor() {
        sensorDistance = opMode.hardwareMap.get(DistanceSensor.class, "ringDetector");
    }

    private void initializedrive() {
        conveyorm = opMode.hardwareMap.get(DcMotorEx.class, "conveyor");

        conveyorm.setDirection(DcMotorSimple.Direction.FORWARD);

        conveyorm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        conveyorm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializedservo() {
        feeder = opMode.hardwareMap.get(Servo.class, "feeder");
        feeder.setPosition(feederClose);
    }

    private double getdistance() {
        return sensorDistance.getDistance(DistanceUnit.CM);
    }

    public void update() {

        if (i > 0 && feederTime.milliseconds() > time * 2.5) {
            feedRing();
            i--;
        }
        setFeederPosition(feederTime.milliseconds() < time);
        if (pauseTime.milliseconds() >= 100) {
            pauseTime.reset();
            distance = getdistance();
        }
        if (distance < 6) {
            if (conveyorTime.milliseconds() >= 1000) {
                full = true;
            }
        } else {
            conveyorTime.reset();
            full = false;
        }
        if (!backMust)
        {
            if (conveyorPower != 0 && !full) {
                if (!stop) {
                    stop = true;
                }
                if (conveyorm.getCurrent(CurrentUnit.AMPS) <= 4 && backOnTime.milliseconds() >= 1000) {
                    if (!backOn) {
                        setConveyorMotorPower(conveyorPower);
                        backOn = true;
                    }
                    timelock = backOnTime.milliseconds();
                } else {
                    if (backOn && (backOnTime.milliseconds() >= (timelock + 500))) {
                        backOnTime.reset();
                        setConveyorMotorPower(-conveyorPower);
                        backOn = false;
                    }
                }
            } else {
                if (stop) {
                    setConveyorMotorPower(0);
                    stop = false;
                    backOn = false;
                }
            }
        } else {
            setConveyorMotorPower(-1);
            backOn = false;
            stop = true;
        }
    }

    private void setFeederPosition(boolean push) {
            if (push) feeder.setPosition(feederOpen);
            else feeder.setPosition(feederClose);
    }

    public void setConveyorPower(double power) {
        conveyorPower = power;
    }

    private void setConveyorMotorPower(double power) {
        conveyorm.setPower(power);
    }
    public void setBackmust(boolean Backmust){
        backMust = Backmust;
    }
    public void feedRing() {
        feederTime.reset();
    }

    public void feedrings() {
        i = 3;
    }
}
