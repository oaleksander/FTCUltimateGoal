package org.firstinspires.ftc.teamcode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
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
    private static int valdown = -1;
    private static int valup = -1;
        private final int rows = 640;
        private final int cols = 480;
    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;
    private static float[] downPos = {4f/8f+offsetX, 4f/8f+offsetY};
    private static float[] upPos = {4f/8f+offsetX,3.5f/8f+offsetY};
    //0 = col, 1 = row

    @Override
        public void runOpMode() throws InstantiationException{
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
                telemetry.addData("Values", valdown+ " "+valup);
                telemetry.addData("Height", rows);
                telemetry.addData("Width", cols);
                telemetry.addData("", contoursList);
                telemetry.update();
                sleep(100);
            }
        }
        static class StageSwitchingPipeline extends OpenCvPipeline {
            Mat all = new Mat();
            Mat yCbCrChan2Mat = new Mat();
            Mat thresholdMat = new Mat();
            public List<MatOfPoint> contoursList = new ArrayList<>();
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

                /* double[] pixMid = thresholdMat.get((int)(input.rows()* downPos[1]), (int)(input.cols()* downPos[0]));//gets value at circle
                valdown = (int)pixMid[0];
                double[] pixLeft = thresholdMat.get((int)(input.rows()* upPos[1]), (int)(input.cols()* upPos[0]));//gets value at circle
                valup = (int)pixLeft[0];

                Point pointdown = new Point((int)(input.cols()* downPos[0]), (int)(input.rows()* downPos[1]));
                Point pointup = new Point((int)(input.cols()* upPos[0]), (int)(input.rows()* upPos[1]));
                Imgproc.circle(all, pointdown,5, new Scalar( 255, 0, 0 ),1 );//draws circle
                Imgproc.circle(all, pointup,5, new Scalar( 255, 0, 0 ),1 );//draws circle
                */switch (stageToRenderToViewport)
                {
                    case THRESHOLD:
                    {
                        return thresholdMat;
                    }

                    case detection:
                    {
                        return all;
                    }

                    case RAW_IMAGE:
                    {
                        return input;
                    }
                    default:
                    {
                        return thresholdMat;
                    }
                }
                //return thresholdMat;
            }
            //
        }
    }

