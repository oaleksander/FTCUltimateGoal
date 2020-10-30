package org.firstinspires.ftc.teamcode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@TeleOp
public class opencvnew extends LinearOpMode {
    OpenCvCamera webcam;
    private final int rows = 640;
    private final int cols = 480;
    //0 = col, 1 = row

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        StageSwitchingPipeline pipeline = new StageSwitchingPipeline();
        webcam.setPipeline(pipeline);
        /*webcam.openCameraDevice();
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);*/
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);
            }
        });
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Mean", pipeline.getMean());
            telemetry.addData("AspectRatio", pipeline.getAspectRatio());
            telemetry.addData("StackSize", pipeline.stackSize);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            telemetry.update();
            sleep(100);
        }
    }

    static class StageSwitchingPipeline extends OpenCvPipeline {

        public enum StackSize {
            ZERO,
            ONE,
            FOUR
        }

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
            Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV);

            Core.inRange(HSVMat, new Scalar(7, (150 + Core.mean(HSVMat).val[1]) / 2, (100 + Core.mean(HSVMat).val[2]) / 2), new Scalar(17, 255, 255), thresholdMat);
            //Core.inRange(HSVMat, new Scalar(7, (110 + Core.mean(HSVMat).val[1]) / 2, (50 + Core.mean(HSVMat).val[2]) / 2), new Scalar(17, 255, 255), thresholdMat);
            Imgproc.erode(thresholdMat, thresholdMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(35, 5)));
            //Imgproc.erode(thresholdMat, thresholdMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(45, 10)));
            Imgproc.dilate(thresholdMat, thresholdMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(35, 15)));
            //Imgproc.dilate(thresholdMat, thresholdMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(45, 15)));

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


               /* Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores
                Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
                Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
               */// HSVMat.copyTo(all);

                /* double[] pixMid = thresholdMat.get((int)(input.rows()* downPos[1]), (int)(input.cols()* downPos[0]));//gets value at circle
                valdown = (int)pixMid[0];
                double[] pixLeft = thresholdMat.get((int)(input.rows()* upPos[1]), (int)(input.cols()* upPos[0]));//gets value at circle
                valup = (int)pixLeft[0];

                Point pointdown = new Point((int)(input.cols()* downPos[0]), (int)(input.rows()* downPos[1]));
                Point pointup = new Point((int)(input.cols()* upPos[0]), (int)(input.rows()* upPos[1]));
                Imgproc.circle(all, pointdown,5, new Scalar( 255, 0, 0 ),1 );//draws circle
                Imgproc.circle(all, pointup,5, new Scalar( 255, 0, 0 ),1 );//draws circle
                */
            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }
                default: {
                    return thresholdMat;
                }

            }//return all;
        }

        public double getMean() {
                return mean;
        }
        public double getAspectRatio() {
            return aspectRatio;
        }
    }
}

