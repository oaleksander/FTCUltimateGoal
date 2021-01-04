package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;

public class TelemetryDebugging implements RobotModule {

    public static double ROBOT_SIDE_LENGTH = 20;
    FtcDashboard dashboard = null;
    Telemetry telemetry;
    TelemetryPacket packet = null;
    long loopCount = 0;
    private LinearOpMode opMode = null;
    private final ElapsedTime measurementTime = new ElapsedTime();

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;

    }

    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        measurementTime.reset();
        loopCount = 0;
        //telemetry = opMode.telemetry;
        telemetry = dashboard.getTelemetry();
        // dashboard.startCameraStream(openCVNode.getWebcam(),0);
    }


    public void update() {
        if (measurementTime.seconds() > 0.25) {
            // telemetry.addData("Status", "Running");
            //telemetry.addData("Loop frequency", 1 / (measurementTime.seconds() / loopCount) + " Hz");

            telemetry.setMsTransmissionInterval(40);
            //telemetry.addLine("Odometry encoders").addData("odYL", odometry.bulkData.getMotorCurrentPosition(0)).addData("odYR", odometry.bulkData.getMotorCurrentPosition(1)).addData("odX", odometry.bulkData.getMotorCurrentPosition(2));
            //  telemetry.addLine("Robot position").addData("y", odometry.getRobotCoordinates().y).addData("x", odometry.getRobotCoordinates().x).addData("head", Math.toDegrees(odometry.getRobotCoordinates().heading));
            //telemetry.addLine("Robot velocity").addData("y", odometry.getRobotVelocity().y).addData("x", odometry.getRobotVelocity().x).addData("head", Math.toDegrees(odometry.getRobotVelocity().z));
            telemetry.addData("Shooter velo", shooter.getCurrentRpm());
            telemetry.addData("Shooter tgt", shooter.getRpmTarget());
            //telemetry.addData("Shooter position", shooter.shooterMotor.getCurrentPosition());
            //telemetry.addData("Shooter current", shooter.shooterMotor.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("conpower", conveyor.conveyorPower);


            //telemetry.addData("OpenCV stack size", openCVNode.getStackSize());

         /*   double by = -odometry.getRobotCoordinates().x / 2.54;
            double bx = odometry.getRobotCoordinates().y / 2.54;
            double l = ROBOT_SIDE_LENGTH / 2;

            double[] bxPoints = {l, -l, -l, l};
            double[] byPoints = {l, l, -l, -l};
            rotatePoints(bxPoints, byPoints, -odometry.getRobotCoordinates().heading);
            for (int i = 0; i < 4; i++) {
                bxPoints[i] += bx;
                byPoints[i] += by;
            }

            packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setFill("black")
                    .fillPolygon(bxPoints, byPoints);
            //fdashboard.sendTelemetryPacket(packet);


            measurementTime.reset();
         */
            loopCount = 0;
            telemetry.update();
        }
        loopCount++;
    }
}
