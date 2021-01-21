package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class OpenCVNodeWebcam extends OpenCVNode {
    @Override
    public void initialize() {
        try {
            int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam.setPipeline(pipeline);
            //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
            webcam.openCameraDeviceAsync(() -> webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT));
        } catch (Exception e) {
            opMode.telemetry.addData("OpenCVNode Error", e.getMessage());
        }
    }
}

