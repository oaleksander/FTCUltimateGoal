package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.HSVLowerBound
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.HSVUpperBound
import org.firstinspires.ftc.teamcode.superclasses.RobotModule
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline

open class OpenCVNodeWebcam : RobotModule() {

    @Config
    @Disabled
    internal object OpenCVConfig {
        @JvmField
        var HSVLowerBound = Scalar(10.0, 160.0, 80.0)
        @JvmField
        var HSVUpperBound = Scalar(22.0, 255.0, 255.0)
    }

    lateinit var webcam: OpenCvCamera
    override fun initialize() {
        try {
            webcam = OpenCvCameraFactory.getInstance().createWebcam(
                opMode.hardwareMap.get(
                    WebcamName::class.java, "Webcam 1"
                ), opMode.hardwareMap.appContext.resources.getIdentifier(
                    "cameraMonitorViewId",
                    "id",
                    opMode.hardwareMap.appContext.packageName
                )
            )

            webcam.setPipeline(pipeline)
            webcam.openCameraDeviceAsync {
                webcam.startStreaming(
                    rows,
                    cols,
                    OpenCvCameraRotation.UPRIGHT
                )
            }
        } catch (e: Exception) {
            opMode.telemetry.addData("OpenCVNode Error", e.message)
        }
    }


    fun stopCam() {
        val finalStackSize = stackSize
        try {
            webcam.closeCameraDeviceAsync { stackSize = finalStackSize }
        } catch (ignored: Exception) {
        }
    }

    fun retrieveResult(): StackSize {
        val stackSize = stackSize
        stopCam()
        return stackSize
    }

    @Volatile
    var stackSize = StackSize.ZERO
    @Volatile
    var mean = 0.0
    @Volatile
    var aspectRatio = 0.0

    enum class StackSize {
        ZERO, ONE, FOUR
    }

    val pipeline = object : OpenCvPipeline() {
        private val stages = Stage.values()
        private var all = Mat()
        private var HSVMat = Mat()
        private var HSVMatMean = Scalar(.0)
        private var thresholdMat = Mat()
        private val structuringElement =
            Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, Size(35.0, 10.0))
        private val crop = Rect(0, 200, rows, cols - 220)
        private val BlurSize = Size(9.0, 9.0)
        private var stageToRenderToViewport = Stage.DETECTION
        override fun onViewportTapped() {
            val currentStageNum = stageToRenderToViewport.ordinal
            var nextStageNum = currentStageNum + 1
            if (nextStageNum >= stages.size) {
                nextStageNum = 0
            }
            stageToRenderToViewport = stages[nextStageNum]
        }

        override fun processFrame(input: Mat): Mat {
            all = input.submat(crop)
            Imgproc.GaussianBlur(all, all, BlurSize, 0.0)
            Imgproc.cvtColor(all, HSVMat, Imgproc.COLOR_RGB2HSV)
            HSVMatMean = Core.mean(HSVMat)
            Core.inRange(
                HSVMat,
                Scalar(
                    HSVLowerBound.`val`[0],
                    (HSVLowerBound.`val`[1] + HSVMatMean.`val`[1]) / 2.0,
                    (HSVLowerBound.`val`[2] + HSVMatMean.`val`[2]) / 2.0
                ),
                HSVUpperBound,
                thresholdMat
            )
            Imgproc.erode(thresholdMat, thresholdMat, structuringElement)
            Imgproc.dilate(thresholdMat, thresholdMat, structuringElement)
            val rect = Imgproc.boundingRect(thresholdMat)
            Imgproc.rectangle(input, crop, Scalar(0.0, 255.0, 0.0), 2)
            Imgproc.rectangle(all, rect, Scalar(0.0, 0.0, 255.0), 2)
            mean = Core.mean(thresholdMat).`val`[0]
            if (mean > 0.1) {
                aspectRatio = rect.width.toDouble() / rect.height.toDouble()
                stackSize = if (aspectRatio > 2.2) StackSize.ONE else StackSize.FOUR
            } else {
                stackSize = StackSize.ZERO
                aspectRatio = 0.0
            }
            return when (stageToRenderToViewport) {
                Stage.DETECTION -> {
                    all
                }
                Stage.RAW_IMAGE -> {
                    input
                }
                Stage.THRESHOLD -> {
                    thresholdMat
                }
            }
        }

    }

    enum class Stage {
        DETECTION,  //includes outlines
        THRESHOLD,  //b&w
        RAW_IMAGE //displays raw view
    }

    companion object {
        const val rows = 640
        const val cols = 480
    }
}