package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;


public class Conveyor implements RobotModule {
    public static DcMotorEx conveyorm = null;
    public static Servo feeder = null;
    public static ElapsedTime conveyortime = new ElapsedTime();
    public static ElapsedTime backcontime = new ElapsedTime();
    static DistanceSensor sensorDistance;
    public static boolean full = false;
    static boolean backon = false, stop = false;
    static boolean backmust = false;
    static double timelock = 0;
    static boolean ispush = false;
    static double conveyorPower = 0;
    private static LinearOpMode opMode = null;
    private static final ElapsedTime timepause = new ElapsedTime();
    private static final ElapsedTime feederTime = new ElapsedTime();
/*    public static final float maxcolor = 255, mincolor = 0;
    static ColorSensor colorSensor;
    float[] hsvValues = new float[3];*/
    double distance = 0;
    private double last_power = 0;
    
    public void setOpMode(LinearOpMode opMode) {
        Conveyor.opMode = opMode;
    }

    /**
     * main int
     */
    public void initialize() {
        feederTime.reset();
        initializecolor();
        initializedrive();
        initializedservo();

    }

    public void reset() {
        feeder.setPosition(0.06);
        conveyorm.setPower(0);
        ispush = false;
        backon = false;
        stop = false;
        conveyorPower = 0;
        i=0;
    }

    /**
     * int color and distanse
     */
    public void initializecolor() {
        //colorSensor = opMode.hardwareMap.get(ColorSensor.class, "ringDetector");
        sensorDistance = opMode.hardwareMap.get(DistanceSensor.class, "ringDetector");
    }

    /**
     * int motors
     */
    public void initializedrive() {
        conveyorm = opMode.hardwareMap.get(DcMotorEx.class, "conveyor");

        conveyorm.setDirection(DcMotorSimple.Direction.FORWARD);

        conveyorm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        conveyorm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * int servo
     */
    public void initializedservo() {
        feeder = opMode.hardwareMap.get(Servo.class, "feeder");
        feeder.setPosition(0.06);
        ispush = false;

    }

    // правильная ли конвертация?
  /*  public float[] getcolor(){
        int colors = colorSensor.argb();
        int a = Color.alpha(colors);
        int r = Color.red(colors);
        int g = Color.green(colors);
        int b = Color.blue(colors);
        Color.colorToHSV(colors, hsvValues);
        return hsvValues;
    }*/
    public double getdistance() {
        return sensorDistance.getDistance(DistanceUnit.CM);
    }
    double time = 125;
    byte i = 0;
    public void update() {

        if (i > 0 && feederTime.milliseconds() > time * 2.5) {
            feedRing();
            i--;
        }
        setFeederPosition(feederTime.milliseconds() < time);
        if (timepause.milliseconds() >= 100) {
            timepause.reset();
            distance = getdistance();
        }
        if (distance < 6) {
            if (conveyortime.milliseconds() >= 1000) {
                full = true;
            }
        } else {
            conveyortime.reset();
            full = false;
        }
        if (!backmust) {
        if (conveyorPower != 0 && !full) {
            if (!stop) {
                stop = true;
            }
            if (conveyorm.getCurrent(CurrentUnit.AMPS) <= 4 && backcontime.milliseconds() >= 1000) {
                if (backon == false) {
                    setConveyorMotorPower(conveyorPower);
                    backon = true;
                }
                timelock = backcontime.milliseconds();
            } else {
                if (backon && (backcontime.milliseconds() >= (timelock + 500))) {
                    backcontime.reset();
                    setConveyorMotorPower(-conveyorPower);
                    backon = false;
                }
            }
        } else {
            if (stop) {
                setConveyorMotorPower(0);
                stop = false;
                backon = false;
            }
        }
    } else {
            //if (backmust) {
                setConveyorMotorPower(-1);
                backon = false;
                stop = true;
            //}
           // else
               // setConveyorMotorPower(0);
    }
    }

    public void setFeederPosition(boolean push) {
            if (push) feeder.setPosition(0.3);
            else feeder.setPosition(0.06);

    }

    public void setConveyorPower(double power) {
        conveyorPower = power;
    }

    private void setConveyorMotorPower(double power) {
        //if (last_power != power) {
            conveyorm.setPower(power);
            //last_power = power;
      //  }
    }
    public void setBackmust(boolean Backmust){
        backmust = Backmust;
    }
    public void feedRing() {
        feederTime.reset();
    }

    public void feedrings() {
        i = 3;
        //feedRing();
    }
}
