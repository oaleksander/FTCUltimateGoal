package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.highH
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.highS
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.highV
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.lowH
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.lowS
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.lowV
import org.firstinspires.ftc.teamcode.superclasses.RobotModule
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline

open class OpenCVNodeWebcam : RobotModule() {

    @Config
    internal object OpenCVConfig {
        @JvmField var lowH = 10.0
        @JvmField var lowS = 160.0
        @JvmField var lowV = 80.0
        @JvmField var highH = 22.0
        @JvmField var highS = 255.0
        @JvmField var highV = 255.0
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
        private val autoTapper = ElapsedTime()
        override fun onViewportTapped() {
            val currentStageNum = stageToRenderToViewport.ordinal
            var nextStageNum = currentStageNum + 1
            if (nextStageNum >= stages.size) {
                nextStageNum = 0
            }
            stageToRenderToViewport = stages[nextStageNum]
        }

        override fun processFrame(input: Mat): Mat {
            val hsvLowerBound = Scalar(lowH, lowS, lowV)
            val hsvUpperBound = Scalar(highH, highS, highV)
            all = input.submat(crop)
            Imgproc.GaussianBlur(all, all, BlurSize, 0.0)
            Imgproc.cvtColor(all, HSVMat, Imgproc.COLOR_RGB2HSV)
            HSVMatMean = Core.mean(HSVMat)
            Core.inRange(
                HSVMat,
                Scalar(
                    hsvLowerBound.`val`[0],
                    (hsvLowerBound.`val`[1] + HSVMatMean.`val`[1]) / 2.0,
                    (hsvLowerBound.`val`[2] + HSVMatMean.`val`[2]) / 2.0
                ),
                hsvUpperBound,
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
            if(autoTapper.seconds()>3) {
                onViewportTapped()
                autoTapper.reset()
            }
            return when (stageToRenderToViewport) {
                Stage.DETECTION -> {
                    all
                }
                Stage.RAW_IMAGE -> {
                    input.submat(crop)
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