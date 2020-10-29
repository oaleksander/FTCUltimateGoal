package org.firstinspires.ftc.teamcode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
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

        @Override
        public void runOpMode() {


            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            webcam.openCameraDevice();
             webcam.setPipeline(new opencvnew.StageSwitchingPipeline());
            webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);

       /* webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                   webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
*/
            waitForStart();

            while (opModeIsActive()) {
                telemetry.update();
                sleep(100);
            }
        }
        static class StageSwitchingPipeline extends OpenCvPipeline {
            Mat all = new Mat();
            Mat yCbCrChan2Mat = new Mat();
            Mat thresholdMat = new Mat();
            List<MatOfPoint> contoursList = new ArrayList<>();
            enum Stage
            {
                detection,//includes outlines
                THRESHOLD,//b&w
                RAW_IMAGE,//displays raw view
            }
            private openCVsample.StageSwitchingPipeline.Stage stageToRenderToViewport = openCVsample.StageSwitchingPipeline.Stage.detection;
            private openCVsample.StageSwitchingPipeline.Stage[] stages = openCVsample.StageSwitchingPipeline.Stage.values();
            @Override
            public void onViewportTapped()
            {
                /*
                 * Note that this method is invoked from the UI thread
                 * so whatever we do here, we must do quickly.
                 */

                int currentStageNum = stageToRenderToViewport.ordinal();

                int nextStageNum = currentStageNum + 1;

                if(nextStageNum >= stages.length)
                {
                    nextStageNum = 0;
                }

                stageToRenderToViewport = stages[nextStageNum];
            }

            @Override
            public Mat processFrame(Mat input)
            {
                Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
                Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores
                Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
                Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                yCbCrChan2Mat.copyTo(all);


                return all;
            }
            //
        }
    }

