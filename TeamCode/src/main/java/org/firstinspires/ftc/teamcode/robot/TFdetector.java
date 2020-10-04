package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.List;

public class TFdetector implements Runnable{

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "4";
    private static final String LABEL_SECOND_ELEMENT = "1";

    private static final String VUFORIA_KEY =
            "AYQyDib/////AAABmWoLWPk9RUbvpT7hIVtMz+KJ7Wgtz7khQCon2wk+3/Mt+oIFV0pwc6vrhxOD2hI8Vh9IvPuTzPC2zBiOYGLIrg9m4lskp19GIKC6mv4bGqkZC0aLiJWnW5SSZRC5inIVhz+PxiQVYqhTVUskF9/ab2xuAFxohYL2mqdxuZPGyqLvpqEwuWWKiecF3S2fkKeQ+3yyryRMQhSd648Tl1NzaRWWXsUDStrFLfCAp+K922bBJaquOpraQ6aP1vu/oPlu7fbxxAcJytVPX81ASdjyPd4gDPp/tYEPk/xs7avDKYvdnBUM/RKxmIVkiWtFuiA5ug2DHM3mPfxm0peM8+2kQVjbGQLRUJdKKmp/QBjCfVOp";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public int recognitionResult = 0;

    public int retrieveResult()
    {
        this.doStop();
        return recognitionResult;
    }

    ElapsedTime uptime = new ElapsedTime();

    private boolean doStop = false;

    public synchronized void doStop() {
        this.doStop = true;
    }

    private synchronized boolean keepRunning() {
        return !this.doStop;
    }

    public LinearOpMode opMode = null;

    @Override
    public void run() {
        doStop = false;
        uptime.reset();

        while (keepRunning() && (!((opMode == null)?WoENrobot.getInstance().opMode.isStopRequested():opMode.isStopRequested()))) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    recognitionResult = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        try {
                            recognitionResult = Integer.parseInt(recognition.getLabel());
                        }
                        catch (NumberFormatException e)
                        {
                            recognitionResult = 0;
                        }
                    }
                }
            }
            if (opMode == null)
            {
                WoENrobot.getInstance().opMode.telemetry.addData("Recognition result: ", recognitionResult);
                WoENrobot.getInstance().opMode.telemetry.update();
            } else {
                opMode.telemetry.addData("Recognition result: ", recognitionResult);
                opMode.telemetry.update();
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void initialize() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        /*
         * Initialize the TensorFlow Object Detection engine.
         */
        int tfodMonitorViewId;
        if (opMode == null)
        {
            tfodMonitorViewId = WoENrobot.getInstance().opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", WoENrobot.getInstance().opMode.hardwareMap.appContext.getPackageName());
        } else {
            tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        }
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        if (tfod != null) {
            tfod.activate();
        }
    }
    public TFdetector(LinearOpMode opMode)
    {
        this.opMode = opMode;
    }
    public TFdetector()
    {
    }
}
