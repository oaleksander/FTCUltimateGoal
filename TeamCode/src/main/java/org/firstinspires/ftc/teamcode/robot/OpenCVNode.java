package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVNode implements RobotModule {
    private static final int rows = 640;
    private static final int cols = 480;
    static OpenCvCamera webcam;
    private static LinearOpMode opMode = null;
    StageSwitchingPipeline pipeline = new StageSwitchingPipeline();

    public void setOpMode(LinearOpMode opMode) {
        OpenCVNode.opMode = opMode;
    }

    public void initialize() {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT));
    }


    public StackSize getStackSize() {
        return pipeline.stackSize;
    }

    public void stopCam() {
        webcam.closeCameraDeviceAsync(() -> {
        });
    }

    public StackSize retrieveResult() {
        StackSize stackSize = getStackSize();
        stopCam();
        return stackSize;
    }

    public double getMean() {
        return pipeline.mean;
    }

    public double getAspectRatio() {
        return pipeline.aspectRatio;
    }

    public enum StackSize {
        ZERO,
        ONE,
        FOUR
    }

    static class StageSwitchingPipeline extends OpenCvPipeline {

        private final Stage[] stages = Stage.values();
        Mat all = new Mat();
        Mat HSVMat = new Mat();
        Mat thresholdMat = new Mat();
        Rect crop = new Rect(0, 200, rows, cols - 220);
        private volatile StackSize stackSize = StackSize.ZERO;
        private double mean = 0.0;
        private double aspectRatio = 0.0;
        private Stage stageToRenderToViewport = Stage.detection;

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            all = input.submat(crop);
            Imgproc.GaussianBlur(all, all, new Size(11, 11), 0);
            Imgproc.cvtColor(all, HSVMat, Imgproc.COLOR_RGB2HSV);

            Core.inRange(HSVMat, new Scalar(7, (150 + Core.mean(HSVMat).val[1]) / 2, (100 + Core.mean(HSVMat).val[2]) / 2), new Scalar(17, 255, 255), thresholdMat);
            Imgproc.erode(thresholdMat, thresholdMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(35, 5)));
            Imgproc.dilate(thresholdMat, thresholdMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(35, 15)));

            Rect rect = Imgproc.boundingRect(thresholdMat);

            Imgproc.rectangle(input, crop, new Scalar(0, 255, 0), 2);
            Imgproc.rectangle(all, rect, new Scalar(0, 0, 255), 2);

            mean = Core.mean(thresholdMat).val[0];

            if (mean > 0.1) {
                aspectRatio = (double) rect.width / (double) rect.height;
                if (aspectRatio > 1.75)
                    stackSize = StackSize.ONE;
                else stackSize = StackSize.FOUR;
            } else {
                stackSize = StackSize.ZERO;
                aspectRatio = 0.0;
            }
            switch (stageToRenderToViewport) {

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }
                case THRESHOLD:
                default: {
                    return thresholdMat;
                }

            }//return all;
        }

        enum Stage {
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

    }
}

