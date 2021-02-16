package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.movement;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.runTime;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;

public class TelemetryDebugging extends RobotModule{

    public static double ROBOT_SIDE_LENGTH = 44.4;
    private final ElapsedTime measurementTime = new ElapsedTime();
    FtcDashboard dashboard = null;
    Telemetry telemetry;
    TelemetryPacket dashboardPacket = null;
    long loopCount = 0;

    @Config
    static class TelemetryConfig {
        public static boolean dashboardTelemetry = true;
        public static int refreshTimeMs = 50;
    }

    @Override
    public void setOpMode(LinearOpMode opMode) {
        super.setOpMode(opMode);
        dashboard = FtcDashboard.getInstance();
        telemetry = opMode.telemetry;

    }

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }


    private void createDashboardRectangle(Pose2D position, String color) {
        double by = -position.x / 2.54;
        double bx = position.y / 2.54;
        double l = ROBOT_SIDE_LENGTH / (2.54 * 2);
        double l2 = l * 435.55 / 444;

        double[] bxPoints = {l, -l, -l, l};
        double[] byPoints = {l2, l2, -l2, -l2};
        rotatePoints(bxPoints, byPoints, -position.heading);
        for (int i = 0; i < 4; i++) {
            bxPoints[i] += bx;
            byPoints[i] += by;
        }

        dashboardPacket.fieldOverlay()
                .setStroke("cyan")
                .setStrokeWidth(2)
                .strokeLine(bx, by, (bxPoints[0] + bxPoints[3]) / 2, (byPoints[0] + byPoints[3]) / 2);
        dashboardPacket.fieldOverlay()
                .setStroke("orange")
                .setStrokeWidth(1)
                .strokeLine(bx, by, (bxPoints[0] + bxPoints[3]) / 2, (byPoints[0] + byPoints[3]) / 2);
        dashboardPacket.fieldOverlay()
                .setStroke(color)
                .setStrokeWidth(1)
                .strokePolygon(bxPoints, byPoints);
    }

    private boolean updaterIsActive = false;

    private final Runnable updateTelemetry = () -> {
        updaterIsActive = true;
        while(opMode.opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            if (measurementTime.milliseconds() > TelemetryConfig.refreshTimeMs) {
                double loopFrequency = loopCount/measurementTime.seconds();
                measurementTime.reset();
                loopCount = 0;


                telemetry.addData("Status", "Running " + runTime.seconds());
                telemetry.addData("Loop frequency",   loopFrequency + " Hz");
                Pose2D robotPosition = odometry.getRobotCoordinates();
                //   telemetry.addLine("Odometry encoders").addData("odYL", odometry.odometerYL.getCurrentPosition()).addData("odYR", odometry.odometerYR.getCurrentPosition()).addData("odX", odometry.odometerX.getCurrentPosition());
                //  telemetry.addLine("Robot position ").addData("Y", robotPosition.y).addData("X", robotPosition.x).addData("Head", Math.toDegrees(robotPosition.heading));
                //   Vector3D velocity = odometry.getRobotVelocity();
                //     telemetry.addLine("Robot velocity ").addData("Y", velocity.y).addData("X", velocity.x).addData("Head", Math.toDegrees(velocity.z));
                telemetry.addLine("Shooter ").addData("Mode", shooter.getShootingMode()).addData("Current", shooter.getEncoderFailureMode()?"Encoder Fail":shooter.getCurrentRpm()).addData("Target", shooter.getRpmTarget());
                //telemetry.addData("conpower", conveyor.conveyorPower);
                //    telemetry.addLine("headings").addData("Encoder",Math.toDegrees(odometry.getEncoderHeading())).addData("IMU1",Math.toDegrees(odometry.getIMUheading_1())).addData("IMU2",Math.toDegrees(odometry.getIMUheading_2()));

                if(TelemetryConfig.dashboardTelemetry) {
                    dashboardPacket = new TelemetryPacket();
                    createDashboardRectangle(robotPosition, "black");
                    if(movement.pathFollowerIsActive())
                        createDashboardRectangle(movement.getCurrentTarget(),"green");
                    dashboardPacket.put("Loop frequency", loopFrequency);
                    dashboardPacket.put("Flywhel RPM",shooter.getCurrentRpm());
                    dashboardPacket.put("Flywhel target",shooter.getRpmTarget());
                    dashboardPacket.put("Status", "Running " + runTime.seconds());
                    dashboard.sendTelemetryPacket(dashboardPacket);
                }


                //telemetry.addData("OpenCV stack size", openCVNode.getStackSize());


                telemetry.update();
            }
            else Thread.yield();
        }
        updaterIsActive = false;
    };

    Thread telemetryUpdater = new Thread(updateTelemetry);

    @Override
    public void initialize() {
        measurementTime.reset();
        loopCount = 0;
        //  telemetry = dashboard.getTelemetry();
        //  telemetry = new MultipleTelemetry(opMode.telemetry,dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(TelemetryConfig.refreshTimeMs);
        //   dashboard.startCameraStream(openCVNode.getWebcam(),0);
    }

    public void start() {
        if(telemetryUpdater.getState()!= Thread.State.NEW) {
            telemetryUpdater.interrupt();
        }
        telemetryUpdater = new Thread(updateTelemetry);
        telemetryUpdater.start();
    }


    public void update() {
        loopCount++;
    }
}
