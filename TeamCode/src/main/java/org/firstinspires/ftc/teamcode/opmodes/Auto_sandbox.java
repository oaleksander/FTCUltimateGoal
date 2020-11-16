package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.openftc.revextensions2.ExpansionHubServo;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.*;


@Autonomous
public class Auto_sandbox extends AutonomousOpMode {
    @Override
    public void main() {
        boolean onShooter = false;
        ExpansionHubServo ss;
    //    shooter.setrpm(1000);
      //  shooter.setspeedlevel(2000);
        shooter.shooterpower(3000);
        //  shooter.setspeedlevel(5000);
        //  conveyor.setConveyorPower(1);
        do {
            onShooter = gamepad1.dpad_down?false : (gamepad1.dpad_up ? true : onShooter);

            //telemetry.addData("getStackSize", openCVNode.getStackSize());
            //telemetry.addData("rpm", shooter.shooterMotor.getVelocity()/0.4);
            spinOnce();
        } while (opModeIsActive());
    }
}
