package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

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
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class GameRecognition {
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private static final String ASSET_NAME = "UltimateGoal";
    private static final String RED_TOWER_GOAL_TARGET = "Red Tower Goal Target";
    private static final String RED_ALLIANCE_TARGET = "Red Alliance Target";
    private static final String BLUE_TOWER_GOAL_TARGET = "Blue Tower Goal Target";
    private static final String BLUE_ALLIANCE_TARGET = "Blue Alliance Target";
    private static final String FRONT_WALL_TARGET = "Front Wall Target";

    public VuforiaTrackables targetsUltimateGoal;
    /**
     * The names of the trackables, ordered to match the Vuforia data files.
     */
    public static final String[] TRACKABLE_NAMES = {
            BLUE_TOWER_GOAL_TARGET,
            RED_TOWER_GOAL_TARGET,
            RED_ALLIANCE_TARGET,
            BLUE_ALLIANCE_TARGET,
            FRONT_WALL_TARGET,
    };
    private static final Map<String, OpenGLMatrix> LOCATIONS_ON_FIELD = new HashMap<>();
    static {
        // Set the position of the perimeter targets with relation to origin (center of field).
        LOCATIONS_ON_FIELD.put(RED_ALLIANCE_TARGET,
                OpenGLMatrix
                        .translation(0, -halfField, mmTargetHeight)
                        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        LOCATIONS_ON_FIELD.put(BLUE_ALLIANCE_TARGET,
                OpenGLMatrix
                        .translation(0, halfField, mmTargetHeight)
                        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        LOCATIONS_ON_FIELD.put(FRONT_WALL_TARGET,
                OpenGLMatrix
                        .translation(-halfField, 0, mmTargetHeight)
                        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));


        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        LOCATIONS_ON_FIELD.put(BLUE_TOWER_GOAL_TARGET,
                OpenGLMatrix
                        .translation(halfField, quadField, mmTargetHeight)
                        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        LOCATIONS_ON_FIELD.put(RED_TOWER_GOAL_TARGET,
                OpenGLMatrix
                        .translation(halfField, -quadField, mmTargetHeight)
                        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
    };

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //#region Vuforia PatternRecog
    public static final String VUFORIA_KEY = "ASVozkX/////AAAAGX+Aimqfn0YRqafZGVaD2MIhBsmxiHLTd4r2XyoV4F/VEvRMnL1mLn7NDtl1onYGhHmJADQR8nt0aX4rZLIAb/7+XxI7LLZV4X0tMBDQyBL6IWcEdgMD63hTKncdP8NsIVJxJOY971/5pVdU50XisgiiAhq3b6D9twKLfGZ9EI2M4XXM0B7BxdA7x7YMD5QcMDf96myKGsPhVlkwz8XvBdbnOvZZg2FoxmhqExRp33AKii1GZRDwvfeco0hEOKusdwOkjbJ5RTJ+9T3fAysvqSovSG8iAWZ98qrG2xop2gK73UPJaY4vj5/1yVBKFMnWt42P931ybmEW1/c5dc8LR1CyD8jCxlgqypf9oCz/q89j";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String LABEL_NO_ELEMENT = "None";

    public VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Telemetry telemetry;

    public void InitGameRecognition(HardwareMap map, Telemetry telemetry, VuforiaLocalizer.CameraDirection cameraDirection){
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        InitVuforia(cameraDirection);
        InitTfod(map);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }
    public void UpdateCameraPreferences(OpenGLMatrix robotFromCamera, VuforiaLocalizer.CameraDirection cameraDirection){
        for (VuforiaTrackable trackable : targetsUltimateGoal) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, cameraDirection);
        }
    }

    public void InitializeFieldTrackables(VuforiaLocalizer vuforia){
        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

    }

    public void BeginTracking() {
        targetsUltimateGoal.activate();
    }
    public void EndTracking(){
        targetsUltimateGoal.deactivate();
    }

    private void GetRawGameObjects(){
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                telemetry.update();
            }
    }
    public void GetGameObjects(){
        for (VuforiaTrackable trackable : targetsUltimateGoal) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
    }

    public void DeinitGameRecognition(){
        tfod.shutdown();
    }

    private void InitVuforia(VuforiaLocalizer.CameraDirection cameraDirection) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = cameraDirection;
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }


    private void InitTfod(HardwareMap map) {
        int tfodMonitorViewId = map.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", map.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}