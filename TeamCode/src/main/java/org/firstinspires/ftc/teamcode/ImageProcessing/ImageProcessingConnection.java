package org.firstinspires.ftc.teamcode.ImageProcessing;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class ImageProcessingConnection {

    private String TAG = "ImageProcessing";

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    public enum LabelProcessing {
        BOLT, BULB, PANEL
    }

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQSKeyz/////AAABmRytc1C8SUgbsE1v0IwnpamOCO0QAw37Ibk6D0UwFr1G5yEjrfFTYrrP9euDdmXfTLpnMlf6kAhcBrs/T3KYkirjTa6Td1Q47QzqDjYr4FXN2FPp9GpfWuVIK4QCBWMGPRHJbu06/iBkcrFyRWk/ZNkw0D2mf0UC2KwUnJzR9UmwI+LAuwm2UdLIj5nsKkzObmqfQK7Fipjrn/zrD6nlkExWd0nTnQXMnR5aAhK73Nj48+WCqw7+p3jtkxbPrQLUC6Jex0HSQxbGMtz2HXgrCGvnj/vQHuRXO7v/kNgJnOzqfP2y2B9KOfnifffudXxucI4+JH8KhB35O6J5D1fnc4XGwf048/UmXn2Ax40O/iqY";


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

    public void InitImageProcessing(HardwareMap hardwareMap) {
        Log.d(TAG, "Start Init");
        initVuforia(hardwareMap);
        initTfod(hardwareMap);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        Log.d(TAG, "Finish Init");

    }

    public LabelProcessing FindLabelProcessing() {
        LabelProcessing labelProcessing = null;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                //telemetry.addData("# Objects Detected", updatedRecognitions.size());
                Log.d(TAG, "# Objects Detected" + "," + updatedRecognitions.size());
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

                    //telemetry.addData(""," ");
                    //telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                    //telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                    //telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                    Log.d(TAG, "Image" + "," + "%s (%.0f %% Conf.)" + "," + recognition.getLabel() + "," + recognition.getConfidence() * 100);
                    Log.d(TAG, "- Position (Row/Col)" + "," + "%.0f / %.0f" + "," + row + "," + col);
                    Log.d(TAG, "- Size (Width/Height)" + "," + "%.0f / %.0f" + "," + width + "," + height);

                    String findLabelProcessing = null;

                    findLabelProcessing = recognition.getLabel();

                    if (findLabelProcessing == LABELS[0])
                    {
                        labelProcessing = LabelProcessing.BOLT;
                    }
                    else if (findLabelProcessing == LABELS[1])
                    {
                        labelProcessing = LabelProcessing.BULB;
                    }
                    else
                    {
                        labelProcessing = LabelProcessing.PANEL;
                    }
                }
            }
        }
        return(labelProcessing);
    }



    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
