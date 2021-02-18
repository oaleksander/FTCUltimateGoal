package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.MathUtil;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.math.Vector2D;
import org.firstinspires.ftc.teamcode.math.Vector3D;
import org.firstinspires.ftc.teamcode.superclasses.Drivetrain;
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule;
import org.firstinspires.ftc.teamcode.superclasses.Odometry;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.firstinspires.ftc.teamcode.misc.motorAccelerationLimiter;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

public class FakeRobot extends MultithreadRobotModule implements Drivetrain, Odometry {

    private final Vector3D maxVelocity = new MecanumDrivetrain().getMaxVelocity();
    boolean started = false;
    private Vector3D targetVelocity = new Vector3D(0, 0, 0);
    private Vector3D targetVelocityFC = new Vector3D(0, 0, 0);
    private Vector3D realVelocityFC = new Vector3D(0, 0, 0);
    private final motorAccelerationLimiter zLimiter = new motorAccelerationLimiter(v -> realVelocityFC.z = v, maxVelocity.z / 0.38);
    private final motorAccelerationLimiter yLimiter = new motorAccelerationLimiter(v -> realVelocityFC.y = v, maxVelocity.y / 0.38);
    private final motorAccelerationLimiter xLimiter = new motorAccelerationLimiter(v -> realVelocityFC.x = v, maxVelocity.x / 0.38);
    private Pose2D currentPosition = new Pose2D(0, 0, 0);
    private final ElapsedTime updateTimer = new ElapsedTime();

    @Override
    public void initialize() {
        targetVelocity = new Vector3D(0, 0, 0);
        realVelocityFC = new Vector3D(0, 0, 0);
        currentPosition = new Pose2D(0, 0, 0);
        updateTimer.reset();
    }

    @Override
    public void start() {
        targetVelocity = new Vector3D(0, 0, 0);
        realVelocityFC = new Vector3D(0, 0, 0);
        currentPosition = new Pose2D(0, 0, 0);
        updateTimer.reset();
        started = false;
    }

    @Override
    public void updateOther() {
        if (!started) {
            started = true;
            updateTimer.reset();
        }
        // realVelocityFC.x = targetVelocityFC.x;
        zLimiter.setVelocity(targetVelocityFC.z);
        yLimiter.setVelocity(targetVelocityFC.y);
        xLimiter.setVelocity(targetVelocityFC.x);
        currentPosition.y += realVelocityFC.y * updateTimer.seconds();
        currentPosition.x += realVelocityFC.x * updateTimer.seconds();
        currentPosition.heading = MathUtil.angleWrap(currentPosition.heading + realVelocityFC.z * updateTimer.seconds());
        updateTimer.reset();
    }

    @Override
    public void setRobotVelocity(double frontways, double sideways, double turn) {
        if (abs(frontways) > maxVelocity.y) frontways = maxVelocity.y * signum(frontways);
        if (abs(sideways) > maxVelocity.x) sideways = maxVelocity.x * signum(sideways);
        if (abs(turn) > maxVelocity.z) turn = maxVelocity.z * signum(turn);

        targetVelocity = new Vector3D(sideways, frontways, turn);
        targetVelocityFC = new Vector3D(new Vector2D(sideways, frontways).rotatedCW(currentPosition.heading), turn);
    }

    @Override
    public Vector3D getMaxVelocity() {
        return maxVelocity;
    }

    @Override
    public Pose2D getRobotCoordinates() {
        return currentPosition;
    }

    @Override
    public void setRobotCoordinates(Pose2D coordinates) {
        currentPosition = coordinates.clone();
    }

    @Override
    public Vector3D getRobotVelocity() {
        return realVelocityFC;
    }
}
