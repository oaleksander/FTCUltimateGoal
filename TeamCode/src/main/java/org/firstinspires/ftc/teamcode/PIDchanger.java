package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name="Concept: Change PID Controller", group = "Examples")
public class PIDchanger extends LinearOpMode {

    // our DC motor.
    DcMotorEx shooter;

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 0;

    public void runOpMode() {
        // get reference to DC motor.
        shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        // wait for start command.
        waitForStart();

        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        PIDFCoefficients pidOrig = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user.
        while(opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.0f, %.0f",
                    pidOrig.p, pidOrig.i, pidOrig.d, pidOrig.f);
          //  telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
          //          pidModified.p, pidModified.i, pidModified.d);
            telemetry.update();
        }
    }
}