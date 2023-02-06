package org.firstinspires.ftc.teamcode.ImageProcessing;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.openCV.GripPipelineBlue;
import org.firstinspires.ftc.teamcode.openCV.GripPipelineGreen;
import org.firstinspires.ftc.teamcode.openCV.GripPipelineRed;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


public class ImageProccessingOpenCV {

    private OpenCvWebcam webcam;
    public boolean findColorObject = false;
    private double maxAreaBlue = 0;
    private double maxAreaRed = 0;
    private double maxAreaGreen = 0;
    private Mat draw = null;
    private Mat output = null;
    private boolean stopRobot = false;
    private Scalar colorRed = null;
    private Scalar colorBlue = null;
    private Scalar colorGreen = null;

    GripPipelineBlue gripPipelineBlue;
    GripPipelineRed gripPipelineRed;
    GripPipelineGreen gripPipelineGreen;
    private LabelProcessing labelProcessing = null;


    public enum LabelProcessing {
        ONE, TWO, THREE
    }

    private final String TAG = "ImageProccessingOpenCV";

    public void Init(HardwareMap hardwareMap)
    {
        Log.d(TAG, "Start Init");

        gripPipelineBlue = new GripPipelineBlue();
        gripPipelineRed = new GripPipelineRed();
        gripPipelineGreen = new GripPipelineGreen();
        colorRed = new Scalar(255, 0, 0);
        colorGreen = new Scalar(0, 255, 0);
        colorBlue = new Scalar(0, 0, 255);



        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */

        webcam.setPipeline(new SamplePipeline());
        webcam.stopRecordingPipeline();

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        Log.d(TAG, "Finish Init");
    }

    public LabelProcessing FindLabelProcessingOpenCV()
    {


        Log.d(TAG, "Frame Count: " + Integer.toString(webcam.getFrameCount()));
        Log.d(TAG, "FPS: " + Float.toString(webcam.getFps()));
        Log.d(TAG, "Total frame time ms: " + Integer.toString(webcam.getTotalFrameTimeMs()));
        Log.d(TAG, "Pipeline time ms: " + Integer.toString(webcam.getPipelineTimeMs()));
        Log.d(TAG, "Overhead time ms: " + Integer.toString(webcam.getOverheadTimeMs()));
        Log.d(TAG, "Theoretical max FPS: " + Integer.toString(webcam.getCurrentPipelineMaxFps()));
        Log.d(TAG, "findColorObject is " + Boolean.toString(findColorObject));

        return (labelProcessing);

    }

    public void LabelProcessingInit()
    {
        labelProcessing = null;
        findColorObject = false;
        stopRobot = false;
    }

    public void StopRobot ()
    {
        stopRobot = true;
        webcam.stopRecordingPipeline();
        webcam.stopStreaming();
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */

            output = null;
            if (true == stopRobot)
            {
                draw = input;
            }
            else
            {
                //Log.d(TAG, "stat frame..");
                findColorObject = false;
                maxAreaBlue = 0;
                maxAreaRed = 0;
                maxAreaGreen = 0;

                output = gripPipelineBlue.process(input.clone());
                output = gripPipelineRed.process(input.clone());
                output = gripPipelineGreen.process(input.clone());

                draw = Mat.zeros(input.size(), CvType.CV_8UC3);
                if ((gripPipelineBlue.getFindContoursOutput() != null) && (!findColorObject)) {
                    int blueSize = gripPipelineBlue.getFindContoursOutput().size();
                    Log.d(TAG, "Blue size is " + blueSize);

                    for (int i = 0; i < blueSize; i++) {
                        // Calculating the area
                        Imgproc.drawContours(draw, gripPipelineBlue.getFindContoursOutput(), i, colorBlue, 2,
                                Imgproc.LINE_8, gripPipelineBlue.hierarchy, 2, new Point());
                        Log.d(TAG, "Blue area [" + i + "] = " + Imgproc.contourArea(gripPipelineBlue.getFindContoursOutput().get(i)));
                        if (Imgproc.contourArea(gripPipelineBlue.getFindContoursOutput().get(i)) > maxAreaBlue) {
                            maxAreaBlue = Imgproc.contourArea(gripPipelineBlue.getFindContoursOutput().get(i));
                        }
                    }
                    if (maxAreaBlue > 200) {
                        findColorObject = true;
                        labelProcessing = LabelProcessing.THREE;
                        Log.d(TAG, "findColorObject blue is true");
                    } else {
                        findColorObject = false;
                        Log.d(TAG, "findColorObject blue is false");
                    }
                }

                if ((gripPipelineRed.getFindContoursOutput() != null) && (!findColorObject)) {
                    int redSize = gripPipelineRed.getFindContoursOutput().size();
                    Log.d(TAG, "Red size is " + redSize);

                    for (int i = 0; i < redSize; i++) {
                        // Calculating the area
                        Imgproc.drawContours(draw, gripPipelineRed.getFindContoursOutput(), i, colorRed, 2,
                                Imgproc.LINE_8, gripPipelineRed.hierarchy, 2, new Point());
                        Log.d(TAG, "Red area [" + i + "] = " + Imgproc.contourArea(gripPipelineRed.getFindContoursOutput().get(i)));
                        if (Imgproc.contourArea(gripPipelineRed.getFindContoursOutput().get(i)) > maxAreaRed) {
                            maxAreaRed = Imgproc.contourArea(gripPipelineRed.getFindContoursOutput().get(i));
                        }
                    }

                    if (maxAreaRed > 400) {
                        findColorObject = true;
                        labelProcessing = LabelProcessing.ONE;
                        Log.d(TAG, "findColorObject red is true");
                    } else {
                        findColorObject = false;
                        Log.d(TAG, "findColorObject red is false");
                    }
                }
                if ((gripPipelineGreen.getFindContoursOutput() != null) && (!findColorObject)) {
                    int greenSize = gripPipelineGreen.getFindContoursOutput().size();
                    Log.d(TAG, "Green size is " + gripPipelineGreen.getFindContoursOutput().size());

                    for (int i = 0; i < greenSize; i++) {
                        // Calculating the area
                        Imgproc.drawContours(draw, gripPipelineGreen.getFindContoursOutput(), i, colorGreen, 2,
                                Imgproc.LINE_8, gripPipelineGreen.hierarchy, 2, new Point());
                        Log.d(TAG, "Green area [" + i + "] = " + Imgproc.contourArea(gripPipelineGreen.getFindContoursOutput().get(i)));
                        if (Imgproc.contourArea(gripPipelineGreen.getFindContoursOutput().get(i)) > maxAreaGreen) {
                            maxAreaGreen = Imgproc.contourArea(gripPipelineGreen.getFindContoursOutput().get(i));
                        }
                    }

                    if (maxAreaGreen > 500) {
                        findColorObject = true;
                        labelProcessing = LabelProcessing.TWO;
                        Log.d(TAG, "findColorObject green is true");
                    } else {
                        findColorObject = false;
                        Log.d(TAG, "findColorObject green is false");
                    }
                }

                /**
                 * NOTE: to see how to get data from your pipeline to your OpMode as well as how
                 * to change which stage of the pipeline is rendered to the viewport when it is
                 * tapped, please see {@link PipelineStageSwitchingExample}
                 */
                output.release();
            }
            return draw;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }


}
