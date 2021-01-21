package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.rpm;

@TeleOp
public class ShooterPIDConfig extends LinearOpMode {
    rpm shooterSystem = new rpm();


    @Override
    public void runOpMode() throws InterruptedException {
        shooterSystem.initialize(this);
        waitForStart();
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.setMsTransmissionInterval(40);
        while(opModeIsActive())
        {
            if(gamepad1.start)
                shooterSystem.setShootingMode(rpm.ShooterMode.HIGHGOAL);
            else if(gamepad1.back)
                shooterSystem.setShootingMode(rpm.ShooterMode.OFF);
            else if(gamepad1.left_stick_button)
                shooterSystem.setShootingMode(rpm.ShooterMode.POWERSHOT);
            if (gamepad1.b)
                shooterSystem.feedRing();
            if (gamepad1.right_stick_button)
                shooterSystem.feedRings();
            telemetry.addData("current",shooterSystem.getCurrentRpm());
            telemetry.addData("target",shooterSystem.getRpmTarget());
            telemetry.update();
            if(gamepad1.y)
                hardwareMap.get(DcMotorEx.class, "shooterMotor").setVelocityPIDFCoefficients(shooterParams.kP,shooterParams.kI,shooterParams.kD,shooterParams.kF);
            shooterSystem.update();
        }
    }

    @Config
    static class shooterParams
    {
        public static double kP = 1;
        public static double kI = 1;
        public static double kD = 1;
        public static double kF = 1;
    }
}
