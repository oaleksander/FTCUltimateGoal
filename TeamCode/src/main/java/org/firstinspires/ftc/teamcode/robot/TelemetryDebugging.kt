package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.robot.WoENrobot.*
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.cos
import kotlin.math.sin

class TelemetryDebugging : MultithreadRobotModule() {
    private val measurementTime = ElapsedTime()
    lateinit var dashboard: FtcDashboard
    lateinit var telemetry: Telemetry
    lateinit var dashboardPacket: TelemetryPacket

    @Volatile
    var loopCount = AtomicInteger(0)

    @Config
    internal object TelemetryConfig {
        var dashboardTelemetry = true
        var refreshTimeMs = 50
    }

    override fun setOpMode(opMode: LinearOpMode) {
        super.setOpMode(opMode)
        dashboard = FtcDashboard.getInstance()
        telemetry = opMode.telemetry
    }

    private fun createDashboardRectangle(position: Pose2D, color: String) {
        val by = -position.x / 2.54
        val bx = position.y / 2.54
        val l = ROBOT_SIDE_LENGTH / (2.54 * 2)
        val l2 = l * 435.55 / 444
        val bxPoints = doubleArrayOf(l, -l, -l, l)
        val byPoints = doubleArrayOf(l2, l2, -l2, -l2)
        rotatePoints(bxPoints, byPoints, -position.heading)
        for (i in 0..3) {
            bxPoints[i] += bx
            byPoints[i] += by
        }
        dashboardPacket.fieldOverlay()
            .setStroke("cyan")
            .setStrokeWidth(2)
            .strokeLine(bx, by, (bxPoints[0] + bxPoints[3]) / 2, (byPoints[0] + byPoints[3]) / 2)
        dashboardPacket.fieldOverlay()
            .setStroke("orange")
            .setStrokeWidth(1)
            .strokeLine(bx, by, (bxPoints[0] + bxPoints[3]) / 2, (byPoints[0] + byPoints[3]) / 2)
        dashboardPacket.fieldOverlay()
            .setStroke(color)
            .setStrokeWidth(1)
            .strokePolygon(bxPoints, byPoints)
    }

    private var updaterIsActive = false
    private val updateTelemetry = Runnable {
        updaterIsActive = true
        while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted) {
            if (measurementTime.milliseconds() > TelemetryConfig.refreshTimeMs) {
                val loopFrequency = loopCount.getAndSet(0) / measurementTime.seconds()
                measurementTime.reset()
                telemetry.addData("Status", "Running " + runTime.seconds())
                telemetry.addData("Loop frequency", "$loopFrequency Hz")
                val robotPosition = odometry.robotCoordinates
                //   telemetry.addLine("Odometry encoders").addData("odYL", odometry.odometerYL.getCurrentPosition()).addData("odYR", odometry.odometerYR.getCurrentPosition()).addData("odX", odometry.odometerX.getCurrentPosition());
                //  telemetry.addLine("Robot position ").addData("Y", robotPosition.y).addData("X", robotPosition.x).addData("Head", Math.toDegrees(robotPosition.heading));
                //   Vector3D velocity = odometry.getRobotVelocity();
                //     telemetry.addLine("Robot velocity ").addData("Y", velocity.y).addData("X", velocity.x).addData("Head", Math.toDegrees(velocity.z));
                //////  //    telemetry.addLine("Shooter ").addData("Mode", shooter.getShootingMode()).addData("Current", shooter.getEncoderFailureMode()?"Encoder Fail":shooter.getCurrentRpm()).addData("Target", shooter.getRpmTarget());
                //telemetry.addData("conpower", conveyor.conveyorPower);
                //    telemetry.addLine("headings").addData("Encoder",Math.toDegrees(odometry.getEncoderHeading())).addData("IMU1",Math.toDegrees(odometry.getIMUheading_1())).addData("IMU2",Math.toDegrees(odometry.getIMUheading_2()));
                if (TelemetryConfig.dashboardTelemetry) {
                    dashboardPacket = TelemetryPacket()
                    createDashboardRectangle(robotPosition, "black")
                    if (movement.pathFollowerIsActive()) createDashboardRectangle(
                        movement.currentTarget,
                        "green"
                    )
                    dashboardPacket.put("Loop frequency", loopFrequency)
                    //    dashboardPacket.put("Flywhel RPM",shooter.getCurrentRpm());
                    //    dashboardPacket.put("Flywhel target",shooter.getRpmTarget());
                    dashboardPacket.put("Status", "Running " + runTime.seconds())
                    dashboard.sendTelemetryPacket(dashboardPacket)
                }


                //telemetry.addData("OpenCV stack size", openCVNode.getStackSize());
                telemetry.update()
            } else Thread.yield()
        }
        updaterIsActive = false
    }
    var telemetryUpdater = Thread(updateTelemetry)
    override fun initialize() {
        measurementTime.reset()
        loopCount.getAndSet(0)
        //  telemetry = dashboard.getTelemetry();
        //  telemetry = new MultipleTelemetry(opMode.telemetry,dashboard.getTelemetry());
        telemetry.msTransmissionInterval = TelemetryConfig.refreshTimeMs
        //   dashboard.startCameraStream(openCVNode.getWebcam(),0);
    }

    override fun start() {
        if (telemetryUpdater.state != Thread.State.NEW) {
            telemetryUpdater.interrupt()
        }
        telemetryUpdater = Thread(updateTelemetry)
        telemetryUpdater.start()
    }

    override fun updateOther() {
        loopCount.getAndIncrement()
    }

    var ROBOT_SIDE_LENGTH = 44.4
    private fun rotatePoints(xPoints: DoubleArray, yPoints: DoubleArray, angle: Double) {
        for (i in xPoints.indices) {
            val x = xPoints[i]
            val y = yPoints[i]
            xPoints[i] = x * cos(angle) - y * sin(angle)
            yPoints[i] = x * sin(angle) + y * cos(angle)
        }
    }
}