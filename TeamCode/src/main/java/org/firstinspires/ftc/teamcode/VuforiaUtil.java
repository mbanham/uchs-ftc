package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.tfod.TfodParameters;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;

public class VuforiaUtil {
    //#region Vuforia PatternRecog
    public static final String VUFORIAKEY = "ASVozkX/////AAAAGX+Aimqfn0YRqafZGVaD2MIhBsmxiHLTd4r2XyoV4F/VEvRMnL1mLn7NDtl1onYGhHmJADQR8nt0aX4rZLIAb/7+XxI7LLZV4X0tMBDQyBL6IWcEdgMD63hTKncdP8NsIVJxJOY971/5pVdU50XisgiiAhq3b6D9twKLfGZ9EI2M4XXM0B7BxdA7x7YMD5QcMDf96myKGsPhVlkwz8XvBdbnOvZZg2FoxmhqExRp33AKii1GZRDwvfeco0hEOKusdwOkjbJ5RTJ+9T3fAysvqSovSG8iAWZ98qrG2xop2gK73UPJaY4vj5/1yVBKFMnWt42P931ybmEW1/c5dc8LR1CyD8jCxlgqypf9oCz/q89j";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String LABEL_NO_ELEMENT = "None";
    static VuforiaTrackable relicTemplate = null;


    private static VuforiaLocalizer vuforia;
    private static TFObjectDetector tfod;
    private static Telemetry telemetry;
    private static HardwareMap hardwareMap;

    public static void Initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        VuforiaUtil.telemetry = telemetry;
        VuforiaUtil.hardwareMap = hardwareMap;

        //get tensorflow unique id
        int cameraMonitorViewId  = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //initialize vuforia camera/detection parameters
        //initialize vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIAKEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        telemetry.addData("init", "vuforia created");
        telemetry.update();
        //load assets to be tracked
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        //initialize tfod

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(cameraMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.numExecutorThreads = 3;
        tfodParameters.maxFrameRate = 5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        relicTrackables.activate();
    }

    public static void stopTFOD() {
        tfod.deactivate();
    }

    public static void startTFOD() {
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT, LABEL_NO_ELEMENT);
        tfod.activate();
    }

    public static ArrayList<ViewObject> getObjectsInFrame() {
        ArrayList<ViewObject> finalObjects = new ArrayList<ViewObject>();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getFtcCameraFromTarget();
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            finalObjects.add(new ViewObject("", trans, rot));
        }
        /*if (tfod != null) {
            List<Recognition> recognitions = tfod.getRecognitions();

            if (recognitions != null) {
                // step through the list of recognitions and display boundary info.
                for (Recognition recognition : recognitions) {

                    ViewObject obj = new ViewObject(recognition.getLabel(), null, null);
                    finalObjects.add(obj);
                    telemetry.addData("DATA", "label (%s)", recognition.getLabel());
                    telemetry.addData("DATA", "  left,top        %.03f , %.03f", recognition.getLeft(), recognition.getTop());
                    telemetry.addData("DATA", "  right,bottom    %.03f , %.03f", recognition.getRight(), recognition.getBottom());
                }
            }else{
                telemetry.addData("VUFORIA ERROR", "failed to get recognitions");
            }
        }else{
            telemetry.addData("VUFORIA ERROR", "TFOD is null");
        }*/
        return finalObjects;
    }
}

