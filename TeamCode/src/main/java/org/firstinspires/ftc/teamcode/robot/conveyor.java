package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;


public class conveyor implements RobotModule {
    public DcMotorEx conveyorm = null;
    public Servo feeder = null;
    public ElapsedTime conveyortime = new ElapsedTime();
    public ElapsedTime backcontime = new ElapsedTime();
    float[] hsvValues = new float[3];
    ColorSensor colorSensor;
    DistanceSensor sensorDistance;
    private LinearOpMode opMode = null;
    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    /** main int */
    public void initialize(){
        initializecolor();
        initializedrive();
        initializedservo();
    }

    /** int color and distanse */
    public void initializecolor(){
        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color");
        sensorDistance = opMode.hardwareMap.get(DistanceSensor.class, "color");
    }
    /** int motors */
    public void initializedrive(){
        conveyorm = opMode.hardwareMap.get(DcMotorEx.class, "conveyor");

        conveyorm.setDirection(DcMotorSimple.Direction.FORWARD);

        conveyorm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        conveyorm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /** int servo */
    public void initializedservo(){
        feeder = opMode.hardwareMap.get(Servo.class, "feeder");
    }
    // правильная ли конвертация?
    public float[] getcolor(){
        int colors = colorSensor.argb();
   /*     int a = Color.alpha(colors);
        int r = Color.red(colors);
        int g = Color.green(colors);
        int b = Color.blue(colors);
     */   Color.colorToHSV(colors, hsvValues);
        return hsvValues;
    }
    public double getdistance(){
        return sensorDistance.getDistance(DistanceUnit.CM);
    }
    public float maxcolor = 255, mincolor = 0;
    boolean full = false;

    static boolean on = false;
    boolean backon = true;
    public void update(){
        if ((mincolor <= getcolor()[0]) && (getcolor()[0]<= maxcolor)) {
            if (conveyortime.milliseconds() >= 1000) {
                full = true;
            }
        }
        else {
            conveyortime.reset();
            full = false;
        }
        if (on && !full) {
            if (conveyorm.getCurrent(CurrentUnit.AMPS)<= 4 && backcontime.milliseconds()>=1000){
                setpowerconveyor(1);
                backon = true;
            }
            else {
                if (backon) {
                    backcontime.reset();
                    setpowerconveyor(-1);
                    backon = false;
                }
            }
        }
        else {
            setpowerconveyor(0);
        }
    }
    public void  setOnConveyor(boolean takeon){
        conveyor.on = takeon;
    }
    boolean ispush = false;
    public void setposclose(boolean push) {
        ispush = push;
        if (push) feeder.setPosition(0.5);
        else feeder.setPosition(1);
    }
    public void setpowerconveyor(double power){
        conveyorm.setPower(power);
    }
    public void runOpMode(){}
}
