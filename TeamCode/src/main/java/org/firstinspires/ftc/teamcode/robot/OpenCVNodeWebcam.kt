package org.firstinspires.ftc.teamcode.robot

import org.firstinspires.ftc.teamcode.superclasses.RobotModule
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraRotation
import org.opencv.core.*
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.jvm.Volatile
import org.opencv.imgproc.Imgproc
import java.lang.Exception

open class OpenCVNodeWebcam : RobotModule() {
    lateinit var webcam: OpenCvCamera
    var pipeline = StageSwitchingPipeline()
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

    val stackSize: StackSize
        get() = pipeline.stackSize

    fun stopCam() {
        val finalStackSize = stackSize
        try {
            webcam.closeCameraDeviceAsync { pipeline.stackSize = finalStackSize }
        } catch (ignored: Exception) {
        }
    }

    fun retrieveResult(): StackSize {
        val stackSize = stackSize
        stopCam()
        return stackSize
    }

    val mean: Double
        get() = pipeline.mean
    val aspectRatio: Double
        get() = pipeline.aspectRatio

    enum class StackSize {
        ZERO, ONE, FOUR
    }

    class StageSwitchingPipeline : OpenCvPipeline() {
        private val stages = Stage.values()
        var all = Mat()
        var HSVMat = Mat()
        var thresholdMat = Mat()
        var crop = Rect(0, 200, rows, cols - 220)

        @Volatile
        var stackSize = StackSize.ZERO
        var mean = 0.0
        var aspectRatio = 0.0
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
            Imgproc.GaussianBlur(all, all, Size(9.0, 9.0), 0.0)
            Imgproc.cvtColor(all, HSVMat, Imgproc.COLOR_RGB2HSV)
            Core.inRange(
                HSVMat,
                Scalar(
                    12.0,
                    (160 + Core.mean(HSVMat).`val`[1]) / 2,
                    (80 + Core.mean(HSVMat).`val`[2]) / 2
                ),
                Scalar(18.0, 255.0, 255.0),
                thresholdMat
            )
            Imgproc.erode(
                thresholdMat,
                thresholdMat,
                Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, Size(35.0, 10.0))
            )
            Imgproc.dilate(
                thresholdMat,
                thresholdMat,
                Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, Size(35.0, 100.0))
            )
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

        internal enum class Stage {
            DETECTION,  //includes outlines
            THRESHOLD,  //b&w
            RAW_IMAGE
            //displays raw view
        }
    }

    companion object {
        const val rows = 640
        const val cols = 480
    }
}