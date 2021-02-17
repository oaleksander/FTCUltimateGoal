package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.CommandSender;
import org.firstinspires.ftc.teamcode.superclasses.Conveyor;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;


public class Conveyor1 extends RobotModule implements Conveyor {
    private final ElapsedTime conveyorTime = new ElapsedTime();
    private final ElapsedTime backOnTime = new ElapsedTime();
    private final ElapsedTime pauseTime = new ElapsedTime();
    private final ElapsedTime BackOnAftertime = new ElapsedTime();

    public static DcMotorEx conveyorm = null;
    private final CommandSender conveyorPowerSender = new CommandSender(p -> conveyorm.setPower(-p));
    public static DistanceSensor sensorDistance;
    private boolean full = false;
    private boolean backOn = false, stop = false;
    private boolean backMust = false;
    private boolean backOnAfter = false;
    private boolean colorLock = false;

    private double timelock = 0;
    private double conveyorPower = 0;
    private double distance = 0;
    private double current = 0;


    public void initialize() {

        initializecolor();
        initializedrive();
    }

    public void start() {

        conveyorm.setPower(0);
        backOn = false;
        stop = false;
        conveyorPower = 0;

    }

    private void initializecolor() {
        sensorDistance = WoENHardware.INSTANCE.getRingDetector();
    }

    private void initializedrive() {
        conveyorm = WoENHardware.INSTANCE.getConveyorMotor();

        conveyorm.setDirection(DcMotorSimple.Direction.FORWARD);

        conveyorm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    private double getdistance() {
        //return 10;
        return sensorDistance.getDistance(DistanceUnit.CM);
    }

    public void update() {

        if (pauseTime.milliseconds() >= 100) {
            pauseTime.reset();
            if (!colorLock)
                distance = getdistance();
            else
                distance = 10;
            current = conveyorm.getCurrent(CurrentUnit.AMPS);
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
                if (current <= 4 && backOnTime.milliseconds() >= 1000) {
                    if (!backOn) {
                        setConveyorMotorPower(conveyorPower);
                        backOn = true;
                    }
                    timelock = backOnTime.milliseconds();
                    BackOnAftertime.reset();
                } else {
                    if (backOn && (backOnTime.milliseconds() >= (timelock + 500))) {
                        backOnTime.reset();
                        setConveyorMotorPower(-conveyorPower);
                        backOn = false;
                    }
                }
            } else {
                if (stop) {
                    if (backOnAfter && BackOnAftertime.milliseconds() < 500)
                        setConveyorMotorPower(-conveyorPower);
                    else {
                        setConveyorMotorPower(0);
                        stop = false;
                        backOn = false;
                    }
                }
            }
        } else {
            setConveyorMotorPower(-1);
            backOn = false;
            stop = true;
        }
    }

    public void setReverseAfterStop(boolean BackOnAfter) {
        backOnAfter = BackOnAfter;
    }

    public void setConveyorPower(double power) {
        conveyorPower = power;
    }

    public void setAutomaticConveyorStopping(boolean doAutomaticConveyorStopping) {
        colorLock = !doAutomaticConveyorStopping;
    }

    private void setConveyorMotorPower(double power) {
        conveyorPowerSender.send(power);
    }

    public void setForceReverse(boolean Backmust) {
        backMust = Backmust;
    }


}
