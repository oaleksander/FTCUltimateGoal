package org.firstinspires.ftc.teamcode.robot

import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera

@Deprecated("")
class OpenCVNodePhonecam : OpenCVNodeWebcam() {
    override fun initialize() {
        try {
            val cameraMonitorViewId = opMode.hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId","id", opMode.hardwareMap.appContext.packageName)
            webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId)
            webcam.setPipeline(pipeline)
            webcam.openCameraDeviceAsync { webcam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT) }
        } catch (e: Exception) {
            opMode.telemetry.addData("OpenCVNode Error", e.message)
        }
    }
}