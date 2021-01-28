package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AI.VuforiaInitializer;
import org.firstinspires.ftc.teamcode.AI.VuforiaInitializer.*;
import org.tensorflow.lite.TensorFlowLite;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.firstinspires.ftc.teamcode.UtilMain.VUFORIA_KEY;

public class RingCountDetection {
    private static final double RING_RATIO = 0.75/5;
    private static final int LIST_SIZE = 30;
    private static final float percentoflist = 0.1f;//at least 10% of the list should be filled before being sure of the ring count

    private static ArrayList<Integer> previousCounts = new ArrayList<Integer>();

    public static RingReturnObject GetRingCount(){
        return GetRingCount(null);
    }
    public static boolean GetRingCountCheck(){
        RingReturnObject ringreturn = GetRingCount();
        if (ringreturn.ring_count != -1 && (ringreturn.listsize / ringreturn.maxlistsize) > percentoflist)
            return true;
        return false;
    }
    public static RingReturnObject GetRingCount(Telemetry telemetry) {
        if (VuforiaInitializer.tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = VuforiaInitializer.tfod.getUpdatedRecognitions();
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

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
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
