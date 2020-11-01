package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class conveyor extends LinearOpMode {
    public DcMotorEx conveyor = null;
    public Servo feeder = null;
    public ElapsedTime conveyortime = new ElapsedTime();
    public ElapsedTime backcontime = new ElapsedTime();
    float[] hsvValues = new float[3];
    ColorSensor colorSensor;
    DistanceSensor sensorDistance;
    /** main int */
    public void initialize(){
        initializecolor();
        initializedrive();
        initializedservo();
    }
    /** int color and distanse */
    public void initializecolor(){
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color");
    }
    /** int motors */
    public void initializedrive(){
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");

        conveyor.setDirection(DcMotorSimple.Direction.FORWARD);

        conveyor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /** int servo */
    public void initializedservo(){
        feeder = hardwareMap.get(Servo.class, "feeder");
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
    public float maxcolor = 255, mincolor = 0;
    boolean full = false;
    public double getdistance(){
        return sensorDistance.getDistance(DistanceUnit.CM);
    }
    boolean on = false;
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
            if (conveyor.getCurrent(CurrentUnit.AMPS)<= 4 && backcontime.milliseconds() < 1000){
                setpowerconveyor(1);
                backcontime.reset();
            }
            else {
                setpowerconveyor(-1);
            }
        }
        else {
            setpowerconveyor(0);
        }
    }
    public void setpowerconveyor(double power){
        conveyor.setPower(power);
    }
    public void runOpMode(){}
}
