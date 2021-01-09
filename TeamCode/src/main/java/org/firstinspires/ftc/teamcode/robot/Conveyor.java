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
    private final ElapsedTime conveyorTime = new ElapsedTime();
    private final ElapsedTime backOnTime = new ElapsedTime();
    private final ElapsedTime pauseTime = new ElapsedTime();

    private LinearOpMode opMode;
    private DcMotorEx conveyorm = null;
    private DistanceSensor sensorDistance;
    private boolean full = false;
    private boolean backOn = false, stop = false;
    private boolean backMust = false;

    private double timelock = 0;
    private double conveyorPower = 0;
    private double distance = 0;

    public void setOpMode(LinearOpMode OpMode) {
        opMode = OpMode;
    }


    public void initialize() {

        initializecolor();
        initializedrive();
    }

    public void reset() {

        conveyorm.setPower(0);
        backOn = false;
        stop = false;
        conveyorPower = 0;

    }

    private void initializecolor() {
        sensorDistance = opMode.hardwareMap.get(DistanceSensor.class, "ringDetector");
    }

    private void initializedrive() {
        conveyorm = opMode.hardwareMap.get(DcMotorEx.class, "shooterMotor2");

        conveyorm.setDirection(DcMotorSimple.Direction.REVERSE);

        conveyorm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        conveyorm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }



    private double getdistance() {
        return sensorDistance.getDistance(DistanceUnit.CM);
    }

    public void update() {


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
        if (!backMust) {
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



    public void setConveyorPower(double power) {
        conveyorPower = power;
    }

    private void setConveyorMotorPower(double power) {
        conveyorm.setPower(power);
    }

    public void setBackmust(boolean Backmust) {
        backMust = Backmust;
    }


}
