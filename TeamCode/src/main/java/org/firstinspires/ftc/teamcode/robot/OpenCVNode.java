package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.WoENrobot;
import org.firstinspires.ftc.teamcode.superclasses.RobotModule;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class OpenCVNode implements RobotModule {
    static OpenCvCamera webcam;
    private final int rows = 640;
    private final int cols = 480;
    StageSwitchingPipeline pipeline = new StageSwitchingPipeline();

    private LinearOpMode opMode = null;

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initialize() {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }


    public StackSize getStackSize() {
        return pipeline.stackSize;
    }

    public void stopcam() {
        webcam.stopStreaming();
    }
    public StackSize retrieveResult() {
        StackSize stackSize = getStackSize();
        webcam.stopStreaming();
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

        private volatile StackSize stackSize = StackSize.ZERO;
        private double mean = 0.0;
        private double aspectRatio = 0.0;

        Mat all = new Mat();
        Mat HSVMat = new Mat();
        Mat thresholdMat = new Mat();

        enum Stage {
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

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
            input.copyTo(all);
            Imgproc.GaussianBlur(all,all,new Size(11, 11), 0);
            Imgproc.cvtColor(all, HSVMat, Imgproc.COLOR_RGB2HSV);

            Core.inRange(HSVMat, new Scalar(7, (150 + Core.mean(HSVMat).val[1]) / 2, (100 + Core.mean(HSVMat).val[2]) / 2), new Scalar(17, 255, 255), thresholdMat);
            Imgproc.erode(thresholdMat, thresholdMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(35, 5)));
            Imgproc.dilate(thresholdMat, thresholdMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(35, 15)));

            Rect rect = Imgproc.boundingRect(thresholdMat);

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

    }
}

