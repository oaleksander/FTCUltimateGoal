package org.firstinspires.ftc.teamcode.robot;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Deprecated
public class OpenCVNodePhonecam extends OpenCVNodeWebcam {
    @Override
    public void initialize() {
        try {
            int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            webcam.setPipeline(pipeline);
            webcam.openCameraDeviceAsync(() -> webcam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT));
        }catch (Exception e) {
            opMode.telemetry.addData("OpenCVNode Error", e.getMessage());
        }
    }
}

