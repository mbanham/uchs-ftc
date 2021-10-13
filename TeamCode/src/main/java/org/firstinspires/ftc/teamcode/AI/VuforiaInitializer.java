package org.firstinspires.ftc.teamcode.AI;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Abstracts.Size;
import org.firstinspires.ftc.teamcode.Utilities.DeviceManager;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class VuforiaInitializer {
    // OBJECT DETECTION
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    public static final String VUFORIA_KEY = "AaxEeJj/////AAABmdLSaHu3GEP4oER0Z4wPyoYCDrpQOx7X1WZ792YNzZwWGHsLEoTDlfqsXqFSF65DRwdrvXt8RCy1oOpTCReV8Mb7NJETFJupTeuqAt9KHsxxSszvgNq6nY4yzLCMWHPWmmh+iKRyJGSKXv4rZ3Z9t1wcPkSZ1p0jBlFX9v4wqGQSumKTKmgpV+133yONI/3EX8UvIJJQMMW65V1SiNaq8xw2NRZMXYU2b8BemEUQTYDmdD1mOoOw5gBSYuNhaWR3JpZ0tGJ2n4CrSQiD+UeGLC1NkA+lFp3XgoYIbHb6p08rdRsvbytOTQ/13fCfiVtpgibzARDbRQQ9PcdTBMphdHjf2ZZmsDG4iDYV2hmUGbMQ";
    static final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    private static final float mmPerInch = 25.4f;
    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
    static final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    static final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    // Constants for perimeter targets
    private static final int field_squares_x = 6;
    //#endregion
    private static final int field_squares_y = 6;
    private static final Size halfField = new Size(field_squares_x * 12 * mmPerInch, field_squares_y * 12 * mmPerInch);
    private static final Size quadField = new Size(field_squares_x * 6 * mmPerInch, field_squares_y * 6 * mmPerInch);
    //#region ROBOT_ATTRIBUTES
    // Attributes that must be modified to pertain to the robot
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    //#endregion
    private static final float TRACKING_BIAS = 0.5f;
    private static final float MIN_RESULT_CONFIDENCE = 0.75f;
    public static CameraManager cameraManager;
    public static Camera camera;
    public static VuforiaLocalizer vuforia;
    public static TFObjectDetector tfod;
    public static VuforiaTrackables targetsUltimateGoal;
    public static List<VuforiaTrackable> allTrackables;
    static ArrayList<Modules> args;
    //#region CLASS MEMBERS
    private static float phoneXRotate = 0;
    private static float phoneYRotate = 0;
    private static float phoneZRotate = 0;

    public static void InitializeVuforia(HardwareMap hardwareMap, Modules... modules) {
        EndTracking();
        args = new ArrayList<Modules>(Arrays.asList(modules));


        DeviceManager.MapWebcam(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        parameters.useExtendedTracking = false;
        cameraManager = ClassFactory.getInstance().getCameraManager();
        parameters.cameraName = DeviceManager.webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Deadline deadline = new Deadline(2, TimeUnit.SECONDS);
        //camera = cameraManager.requestPermissionAndOpenCamera(deadline, parameters.cameraName, null);

        //  Instantiate the Vuforia engine
        if (args.contains(Modules.ObjectDetection)) {
            System.out.println("OBJECT_DETECTION");
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = MIN_RESULT_CONFIDENCE;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        }
        if (args.contains(Modules.RobotTransformDetection)) {
            System.out.println("TRANSFORM_DETECTION");
            VuforiaTrackables targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
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

            allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(targetsUltimateGoal);

            redAllianceTarget.setLocation(OpenGLMatrix
                    .translation(0, -halfField.height, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            blueAllianceTarget.setLocation(OpenGLMatrix
                    .translation(0, halfField.height, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
            frontWallTarget.setLocation(OpenGLMatrix
                    .translation(-halfField.width, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

            // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
            blueTowerGoalTarget.setLocation(OpenGLMatrix
                    .translation(halfField.width, quadField.height, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
            redTowerGoalTarget.setLocation(OpenGLMatrix
                    .translation(halfField.width, -quadField.height, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


            if (CAMERA_CHOICE == BACK) {
                phoneYRotate = -90;
            } else {
                phoneYRotate = 90;
            }

            // Rotate the phone vertical about the X axis if it's in portrait mode
            if (PHONE_IS_PORTRAIT) {
                phoneXRotate = 90;
            }

            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            /**  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            }


        }


    }

    public static void BeginTracking() {
        if (args.contains(Modules.ObjectDetection)) {
            System.out.println("OBJECT_DETECTION");
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0 / 9.0);
        }
        if (args.contains(Modules.RobotTransformDetection)) {
            System.out.println("TRANSFORM_DETECTION");
            targetsUltimateGoal.activate();
        }
    }

    public static void EndTracking() {
        if (args != null) {
            if (args.contains(Modules.ObjectDetection)) {
                tfod.deactivate();
            }
            if (args.contains(Modules.RobotTransformDetection)) {
                targetsUltimateGoal.deactivate();
            }
            args = null;
        }
    }

    public enum Modules {
        ObjectDetection,
        RobotTransformDetection
    }
}
