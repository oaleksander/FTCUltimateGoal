package org.firstinspires.ftc.teamcode.robot

import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera2

@Deprecated("")
class OpenCVNodePhonecam : OpenCVNodeWebcam() {
    override fun initialize() {
        try {
            webcam = OpenCvCameraFactory.getInstance().createInternalCamera2(
                OpenCvInternalCamera2.CameraDirection.BACK,
                opMode.hardwareMap.appContext.resources.getIdentifier(
                    "cameraMonitorViewId",
                    "id",
                    opMode.hardwareMap.appContext.packageName
                )
            ) as OpenCvInternalCamera2
            webcam.setPipeline(pipeline)
            webcam.openCameraDeviceAsync {
                webcam.startStreaming(
                    rows,
                    cols,
                    OpenCvCameraRotation.SIDEWAYS_LEFT
                )
            }
        } catch (e: Exception) {
            opMode.telemetry.addData("OpenCVNode Error", e.message)
        }
    }
}