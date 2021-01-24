package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.CommandSender;
@Deprecated
@Disabled
public class Conveyor2 {
    private LinearOpMode opMode;
    private DcMotorEx conveyor = null;
    private final CommandSender conveyorPowerSender = new CommandSender(p -> conveyor.setPower(-p));
    private DistanceSensor sensorDistance;

    public void initialize(){
        initializecolor();
        initializedrive();
    }
    private void initializecolor() {
        sensorDistance = opMode.hardwareMap.get(DistanceSensor.class, "ringDetector");
    }

    private void initializedrive() {
        conveyor = opMode.hardwareMap.get(DcMotorEx.class, "odometerYL");
        conveyor.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getdistance(){
        return sensorDistance.getDistance(DistanceUnit.CM);
    }
    private double getAMPS(){
        return conveyor.getCurrent(CurrentUnit.AMPS);
    }
    public void update() {

    }
}
