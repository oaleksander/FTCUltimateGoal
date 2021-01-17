package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.opencv.photo.TonemapReinhard;

import static org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode;
import static org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter;

public class TelemetryDebugging implements RobotModule {

    public static double ROBOT_SIDE_LENGTH = 44.4;
    FtcDashboard dashboard = null;
    Telemetry telemetry;
    TelemetryPacket packet = null;
    long loopCount = 0;
    int refreshTimeMs = 33;
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
        telemetry = opMode.telemetry;
      //  telemetry = dashboard.getTelemetry();
      //  telemetry = new MultipleTelemetry(opMode.telemetry,dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(refreshTimeMs);
      //   dashboard.startCameraStream(openCVNode.getWebcam(),0);
    }


    public void update() {
        if (measurementTime.milliseconds() > refreshTimeMs) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Loop frequency", 1 / (measurementTime.seconds() / loopCount) + " Hz");
           Pose2D robotPosition = odometry.getRobotCoordinates();
           // telemetry.addLine("Odometry encoders").addData("odYL", odometry.odometerYL.getCurrentPosition()).addData("odYR", odometry.odometerYR.getCurrentPosition()).addData("odX", odometry.odometerX.getCurrentPosition());
            telemetry.addLine("Robot position ").addData("Y", robotPosition.y).addData("X", robotPosition.x).addData("Head", Math.toDegrees(robotPosition.heading));
            Vector3D velocity = odometry.getRobotVelocity();
         //   telemetry.addLine("Robot velocity ").addData("Y", velocity.y).addData("X", velocity.x).addData("Head", Math.toDegrees(velocity.z));
            telemetry.addLine("Shooter ").addData("Mode",shooter.getShootingMode()).addData("Current", shooter.getCurrentRpm()).addData("Target",shooter.getRpmTarget());
          //telemetry.addData("conpower", conveyor.conveyorPower);
        //    telemetry.addData("Shooter Velo",shooter.getCurrentRpm());


            //telemetry.addData("OpenCV stack size", openCVNode.getStackSize());

            double by = -robotPosition.x / 2.54;
            double bx = robotPosition.y / 2.54;
            double l = ROBOT_SIDE_LENGTH / (2.54*2);
            double l2 = l*435.55/444;

            double[] bxPoints = {l, -l, -l, l};
            double[] byPoints = {l2, l2, -l2, -l2};
            rotatePoints(bxPoints, byPoints, -odometry.getRobotCoordinates().heading);
            for (int i = 0; i < 4; i++) {
                bxPoints[i] += bx;
                byPoints[i] += by;
            }

            packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStroke("cyan")
                    .setStrokeWidth(2)
                    .strokeLine(bx,by,(bxPoints[0]+bxPoints[3])/2,(byPoints[0]+byPoints[3])/2);
            packet.fieldOverlay()
                    .setStroke("orange")
                    .setStrokeWidth(1)
                    .strokeLine(bx,by,(bxPoints[0]+bxPoints[3])/2,(byPoints[0]+byPoints[3])/2);
            packet.fieldOverlay()
                    .setStroke("black")
                    .setStrokeWidth(1)
                    .strokePolygon(bxPoints, byPoints);
            dashboard.sendTelemetryPacket(packet);


            measurementTime.reset();
            loopCount = 0;
            telemetry.update();
        }
        loopCount++;
    }
}
