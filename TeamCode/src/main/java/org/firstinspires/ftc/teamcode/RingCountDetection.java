package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.firstinspires.ftc.teamcode.UtilMain.VUFORIA_KEY;

public class RingCountDetection {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";


    private static final double RING_RATIO = 0.75/5;
    private static final int LIST_SIZE = 30;

    private static VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private static TFObjectDetector tfod;

    private static ArrayList<Integer> previousCounts = new ArrayList<Integer>();

    public static void Initialize(HardwareMap hardwareMap){
        initVuforia();
        initTfod(hardwareMap);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }
    }

    public static RingReturnObject GetRingCount(){
        return GetRingCount(null);
    }
    public static RingReturnObject GetRingCount(Telemetry telemetry) {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (telemetry != null)
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int suspected_ring_count = 0;
                for (Recognition recognition : updatedRecognitions) {

                    float best_val = Float.MAX_VALUE;
                    //ring_count = stupid2(recognition);
                    for (int j = 1; j < 4; j++) {
                        float val = RatioDeviation(recognition, j);
                        //System.out.println("DIRECTION=" + getDeviationDirection(recognition));
                        if (val < best_val) {
                            best_val = val;
                            suspected_ring_count = j;
                        }
                    }
                }
                if (updatedRecognitions.size() > 0) {
                    if (previousCounts.size() >= LIST_SIZE) {
                        previousCounts.remove(0);
                       //System.out.println("removing last element");
                    }
                    previousCounts.add(suspected_ring_count);
                    //System.out.println("added RING_COUNT=" + ring_count + " size=" + previousCounts);
                }


                Collections.sort(previousCounts);

                int max_chain = 0;
                int current_val = 0;
                int current_chain = 0;
                int ring_number = -1;

                for (int ring : previousCounts) {
                    if (current_val == ring) {
                        current_chain++;
                        //System.out.println("same value " + ring + " | chain=" + current_chain);
                        if (current_chain > max_chain) {
                            ring_number = current_val;
                            max_chain = current_chain;
                        }
                    } else {
                        //System.out.println("different value " + ring);
                        current_val = ring;
                        current_chain = 1;
                    }
                }


                //System.out.println("MODE=" + ring_number);
                if (telemetry != null)
                    telemetry.update();

                return new RingReturnObject(ring_number, previousCounts.size(), LIST_SIZE);

            }
        }
        return new RingReturnObject("Skipped routine. TFOD possibly failed to initialize");
    }
    private static float RatioDeviation(Recognition recognition, int rings){
        System.out.println("-------" + recognition.estimateAngleToObject(AngleUnit.DEGREES));
        return Math.abs(((recognition.getWidth()/recognition.getHeight()) - ((float)RING_RATIO*rings))/((float)RING_RATIO*rings));
    }
    private static void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private static void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
class RingReturnObject{
    int ring_count;
    int listsize;
    int maxlistsize;
    private String why = "";

    public RingReturnObject(int ring_count, int listsize, int maxlistsize){
        this.ring_count = ring_count;
        this.listsize = listsize;
    }
    public RingReturnObject(String message){
        this.why = message;
    }
    public String why(){
        return why;
    }
}
