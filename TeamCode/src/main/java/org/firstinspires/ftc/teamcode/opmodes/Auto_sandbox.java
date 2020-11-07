package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.misc.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.Conveyor;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import org.firstinspires.ftc.teamcode.robot.rpm;
import org.firstinspires.ftc.teamcode.superclasses.AutonomousOpMode;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.conveyor;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.spinOnce;

@Autonomous
public class Auto_sandbox extends AutonomousOpMode {
    @Override
    public void main() {
        shooter.setrpm(5000);
        //  shooter.setspeedlevel(5000);
        //  shooter.onshooter(true);
        //  conveyor.setConveyorPower(1);
        do {
            telemetry.addData("getStackSize", WoENrobot.openCVNode.getStackSize());
            telemetry.addData("rpm", rpm.rpm2);
            telemetry.addData("powercorect", rpm.power2);
            telemetry.addData("amp", Conveyor.conveyorm.getCurrent(CurrentUnit.AMPS));
            spinOnce();
        } while (opModeIsActive());
    }
}
