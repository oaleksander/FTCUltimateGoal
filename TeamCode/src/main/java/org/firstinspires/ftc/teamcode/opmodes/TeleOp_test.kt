package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.misc.ButtonSwitch
import org.firstinspires.ftc.teamcode.misc.SinglePressButton
import org.firstinspires.ftc.teamcode.robot.WoENrobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.forceInitRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry
import org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter
import org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot
import org.firstinspires.ftc.teamcode.robot.WoENrobot.wobbleManipulator
import org.firstinspires.ftc.teamcode.superclasses.Shooter

@TeleOp
class TeleOp_test : LinearOpMode() {
    override fun runOpMode() {
        WoENrobot.initRobot(this)
        startRobot()
        /* Auto Shooting */
        val autoShootingPresser = SinglePressButton { gamepad1.touchpad }
        /* Wobble */
        val grabWobbleSwitch = ButtonSwitch { gamepad1.b }
        /* Conveyor */
        val conveyorOnOffSwitch = ButtonSwitch { gamepad1.back }
        /* Shooter */
        val shooterOnOffSwitch = ButtonSwitch { gamepad1.start }
        val shooterSpeedSwitch = ButtonSwitch { gamepad1.dpad_left }
        val threeRingPresser = SinglePressButton { gamepad1.right_stick_button }

        while (opModeIsActive()) {
            /* Auto Shooting */
            if(autoShootingPresser.get()) {
                if (movement.pathFollowerIsActive()) movement.stopPathFollowing()
                else MovementMacros.shootHighGoalAsync()
            }
            if(!movement.pathFollowerIsActive()) {
                /* Wobble */
                wobbleManipulator.grabWobble(grabWobbleSwitch.get())
                wobbleManipulator.upmediumdown(gamepad1.y, gamepad1.x) // correct
                /* Conveyor */
                WoENrobot.conveyor.enableConveyor = conveyorOnOffSwitch.get()
                /* Shooter */
                shooter.shootingMode = if (shooterOnOffSwitch.get()) if (shooterSpeedSwitch.get()) Shooter.ShooterMode.POWERSHOT
                else Shooter.ShooterMode.HIGHGOAL
                else Shooter.ShooterMode.OFF
                WoENrobot.conveyor.forceReverse = gamepad1.dpad_right
                if (gamepad1.a) shooter.feedRing()
                if (threeRingPresser.get()) shooter.feedRings()
            }
            /* Drivetrain */
            movement.humanSetVelocity(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble(),
                                      if (gamepad1.right_bumper) 0.25 else gamepad1.right_trigger.toDouble() - if (gamepad1.left_bumper) 0.25 else gamepad1.left_trigger.toDouble())
        }
    }
}