package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.misc.ButtonSwitch;
import org.firstinspires.ftc.teamcode.misc.SinglePressButton;
import org.firstinspires.ftc.teamcode.robot.rpm;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.drivetrain;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.forceInitRobot;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.spinOnce;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator2;

@TeleOp(name = "Teleop TEST/PRACTICE", group = "Debugging")

public class Tele_test extends LinearOpMode {

    boolean buttonAwasPressed = false;

    @Override
    public void runOpMode() {
        forceInitRobot(this);
        startRobot();
        ButtonSwitch buttonAswitch = new ButtonSwitch();
        ButtonSwitch smartModeSwitch = new ButtonSwitch();
        ButtonSwitch buttonStartswitch = new ButtonSwitch();
        SinglePressButton threeRingPresser = new SinglePressButton();

        //odometry.setRobotCoordinates(new Pose2D(0, 0, 0));
        odometry.setRobotCoordinates(new Pose2D(93.75 * 1 + 31.25 * (-1), -156.5, 0));
        //shooter.setShootersetings(4444, 2000);
        buttonAswitch.isTriggered(true);
        buttonAswitch.isTriggered(false);
        // shooter.onshooter(true);
        // delay(5000);
        //conveyor.feedrings();
        //wobbleManipulator2.changepos(WobbleManipulator2.positions.down);
        // wobbleManipulator2.setAngle(1);
        while (opModeIsActive()) {
           // drivetrain.setSmartMode(smartModeSwitch.isTriggered(gamepad1.left_stick_button));
            wobbleManipulator2.setposclose(buttonAswitch.isTriggered(gamepad1.a));
            wobbleManipulator2.upmediumdown(gamepad1.y, gamepad1.x); // correct
            shooter.setShootingMode(buttonStartswitch.isTriggered(gamepad1.start)?
                    rpm.ShooterMode.HIGHGOAL
                    :rpm.ShooterMode.OFF);
            if (threeRingPresser.isTriggered(gamepad1.b))
                shooter.feedRings();
            drivetrain.setRobotVelocity(calculateDrivingVelocity());
            spinOnce();
        }
    }

    Vector3D calculateDrivingVelocity() {
        double turn = 0;
        double y;
        double x;
        if (gamepad1.left_bumper) turn -= 0.25;
        else turn -= gamepad1.left_trigger;
        if (gamepad1.right_bumper) turn += 0.25;
        else turn += gamepad1.right_trigger;
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        if (gamepad1.dpad_up)
            y += 1;
        if (gamepad1.dpad_down)
            y = -1;
        if (gamepad1.dpad_left)
            x = -1;
        if (gamepad1.dpad_right)
            x += 1;
        return new Vector3D(x, y, turn).multiply(drivetrain.getMaxVelocity());
    }
}
