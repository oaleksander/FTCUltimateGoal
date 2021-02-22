package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.math.MathUtil
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.math.Vector2D
import org.firstinspires.ftc.teamcode.math.Vector3D
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import org.firstinspires.ftc.teamcode.superclasses.Odometry

class ThreeWheelOdometry : MultithreadRobotModule(), Odometry {
    private val yWheelPairRadiusCm = 18.2425
    private var worldPosition = Pose2D()
    private var angleOffset = 0.0
    private lateinit var imu1: BNO055IMU
    private lateinit var imu2: BNO055IMU
    private lateinit var odometerYL: DcMotorEx
    private lateinit var odometerYR: DcMotorEx
    private lateinit var odometerX: DcMotorEx
    private var odometryWheelDiameterCm = OdometryConfig.forwardMultiplier * 4.8
    private var odometryCountsPerCM = 1440 / (odometryWheelDiameterCm * Math.PI)
    private var odometryCMPerCounts = odometryWheelDiameterCm * Math.PI / 1440
    private var odometerXcenterOffset =
        -21.7562349 * odometryCountsPerCM * Math.cos(Math.toRadians(51.293002))
    private var radiansPerEncoderDifference =
        OdometryConfig.headingMultiplier * (odometryCMPerCounts / (yWheelPairRadiusCm * 2))
    private var IMUoffset1 = 0f
    private var IMUoffset2 = 0f
    private var encoderHeadingCovariance = 0.0
    private var YL_old = 0
    private var YR_old = 0
    private var X_old = 0
    private val YWheelPairCenterOffset = Vector2D(0.0, 6.40008)
    private val IMU1AccessTimer = ElapsedTime()
    private val IMU2AccessTimer = ElapsedTime()

    @Config
    internal object OdometryConfig {
        @JvmField
        var forwardMultiplier = 1.00
        @JvmField
        var headingMultiplier = 1.003851129713462
        @JvmField
        var doUseIMU = false
    }

    private var doUseIMU_local = OdometryConfig.doUseIMU
    private fun calculateHeading(): Double {
        if (doUseIMU_local && false) {
            val angleDivergence = MathUtil.angleWrap(
                encoderHeading - MathUtil.angleAverage(
                    currentIMU1Heading,
                    currentIMU2Heading
                )
            )
            encoderHeadingCovariance = angleDivergence * (1.0 / 2.0)
        }
        return MathUtil.angleWrap(encoderHeading - angleOffset - encoderHeadingCovariance)
    }

    private val encoderHeading: Double
        get() = getEncoderHeading(
            odometerYL.currentPosition.toDouble(),
            odometerYR.currentPosition.toDouble()
        )

    private fun getEncoderHeading(L: Double, R: Double): Double {
        return (L - R) * radiansPerEncoderDifference
    }

    private fun initIMU() {
        imu1 = WoENHardware.controlHubIMU
        val parameters1 = BNO055IMU.Parameters()
        parameters1.accelRange = BNO055IMU.AccelRange.G2
        parameters1.gyroRange = BNO055IMU.GyroRange.DPS500
        parameters1.mode = BNO055IMU.SensorMode.IMU
        parameters1.calibrationDataFile = "BNO055IMUCalibration_1.json"
        imu1.initialize(parameters1)
        imu2 = WoENHardware.imu2
        val parameters2 = BNO055IMU.Parameters()
        parameters2.accelRange = BNO055IMU.AccelRange.G2
        parameters2.gyroRange = BNO055IMU.GyroRange.DPS500
        parameters2.mode = BNO055IMU.SensorMode.IMU
        parameters2.calibrationDataFile = "BNO055IMUCalibration_2.json"
        imu2.initialize(parameters2)
        encoderHeadingCovariance = 0.0
        IMU1AccessTimer.reset()
        IMU2AccessTimer.reset()
    }

    private var currentIMU1Heading = 0.0
    private fun updateCHIMUHeading() {
        currentIMU1Heading = if (doUseIMU_local) MathUtil.angleWrap(
            (-imu1.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.RADIANS
            ).firstAngle - IMUoffset1).toDouble()
        ) else -0.0
    }

    private var currentIMU2Heading = 0.0
    private fun updateEHIMUHeading() {
        currentIMU2Heading = if (doUseIMU_local) MathUtil.angleWrap(
            (-imu2.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.RADIANS
            ).firstAngle - IMUoffset2).toDouble()
        ) else -0.0
    }

    override fun start() {
        if (doUseIMU_local) {
            IMUoffset1 = -imu1.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.RADIANS
            ).firstAngle
            IMUoffset2 = -imu2.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZXY,
                AngleUnit.RADIANS
            ).firstAngle
        }
    }

    override fun updateControlHub() {
        if (doUseIMU_local && IMU1AccessTimer.seconds() > 1) {
            updateCHIMUHeading()
            IMU1AccessTimer.reset()
        }
    }


    override fun updateExpansionHub() {
        if (doUseIMU_local && IMU2AccessTimer.seconds() > 1) {
            updateEHIMUHeading()
            IMU2AccessTimer.reset()
        }

        currentYLVelocity = odometerYL.velocity
        currentYLVelocity = odometerYR.velocity
        currentYLVelocity = odometerX.velocity
        calculatePosition(worldPosition)
    }

    override fun updateOther() {
    }

    @Synchronized
    fun calculatePosition(initialPose: Pose2D) {
        worldPosition = initialPose
        val deltaWorldHeading = MathUtil.angleWrap(calculateHeading() - worldPosition.heading)
        var deltaPosition = Vector2D(
            (odometerX.currentPosition - X_old).toDouble() - deltaWorldHeading * odometerXcenterOffset,
            (odometerYL.currentPosition - YL_old + (odometerYR.currentPosition - YR_old)).toDouble() / 2
        )
        if (deltaWorldHeading != 0.0) {   //if deltaAngle = 0 radius of the arc is = Inf which causes model degeneracy
            val arcAngle = deltaWorldHeading * 2
            val arcRadius = deltaPosition.radius() / arcAngle
            deltaPosition = Vector2D(
                arcRadius * (1 - Math.cos(arcAngle)),
                arcRadius * Math.sin(arcAngle)
            ).rotatedCW(deltaPosition.acot())
        }
        worldPosition = worldPosition.plus(
            Pose2D(
                deltaPosition.rotatedCW(worldPosition.heading),
                deltaWorldHeading
            )
        )
        YL_old = odometerYL.currentPosition
        YR_old = odometerYR.currentPosition
        X_old = odometerX.currentPosition
    }

    override fun initialize() {
        doUseIMU_local = OdometryConfig.doUseIMU
        if (doUseIMU_local) initIMU()
        radiansPerEncoderDifference =
            OdometryConfig.headingMultiplier * (odometryCMPerCounts / (yWheelPairRadiusCm * 2))
        odometryWheelDiameterCm = OdometryConfig.forwardMultiplier * 4.8
        odometryCountsPerCM = 1440 / (odometryWheelDiameterCm * Math.PI)
        odometryCMPerCounts = odometryWheelDiameterCm * Math.PI / 1440
        odometerXcenterOffset =
            -21.7562349 * odometryCountsPerCM * Math.cos(Math.toRadians(51.293002))
        odometerYL = WoENHardware.odometerYL
        odometerYR = WoENHardware.odometerYR
        odometerX = WoENHardware.odometerX
        odometerYL.direction = DcMotorSimple.Direction.FORWARD
        odometerYR.direction = DcMotorSimple.Direction.REVERSE
        odometerX.direction = DcMotorSimple.Direction.REVERSE
        odometerYL.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        odometerYR.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        odometerX.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        odometerYL.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        odometerYR.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        odometerX.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        YL_old = odometerYL.currentPosition
        YR_old = odometerYR.currentPosition
        X_old = odometerX.currentPosition
    }

    /**
     * Returns the robot's global coordinates
     *
     * @return robot position in (x,y,heading) form
     */
    override fun getRobotCoordinates(): Pose2D {
        val poseTranslation = Vector2D(
            worldPosition.x * odometryCMPerCounts,
            worldPosition.y * odometryCMPerCounts
        ).minus(YWheelPairCenterOffset.rotatedCW(worldPosition.heading))
        return Pose2D(poseTranslation, worldPosition.heading)
    }

    override fun setRobotCoordinates(coordinates: Pose2D) {
        YL_old = odometerYL.currentPosition
        YR_old = odometerYR.currentPosition
        X_old = odometerX.currentPosition
        angleOffset = MathUtil.angleWrap(calculateHeading() + angleOffset - coordinates.heading)
        calculatePosition(
            Pose2D(
                Vector2D(
                    coordinates.x * odometryCountsPerCM,
                    coordinates.y * odometryCountsPerCM
                ).plus(
                    YWheelPairCenterOffset.times(odometryCountsPerCM)
                        .rotatedCW(worldPosition.heading)
                ),
                coordinates.heading
            )
        )
    }

    private var currentYLVelocity: Double = .0
    private var currentYRVelocity: Double = .0
    private var currentXVelocity: Double = .0

    override fun getRobotVelocity(): Vector3D {
        val angularVelocity = getEncoderHeading(currentYLVelocity, currentYRVelocity)
        return Vector3D(
            Vector2D(
                (currentXVelocity - angularVelocity * odometerXcenterOffset) * odometryCMPerCounts,
                (currentYLVelocity + currentYRVelocity) * odometryCMPerCounts / 2
            ).rotatedCW(worldPosition.heading), angularVelocity
        )
    }
}