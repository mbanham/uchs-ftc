package org.firstinspires.ftc.teamcode;/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Utilities.Point;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.UtilHolonomic.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.UtilHolonomic.TOLERANCE_WHEEL_POS;
import static org.firstinspires.ftc.teamcode.UtilMain.VUFORIA_KEY;

/**
 * This 2020-2021 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the ULTIMATE GOAL FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * There are a total of five image targets for the ULTIMATE GOAL game.
 * Three of the targets are placed in the center of the Red Alliance, Audience (Front),
 * and Blue Alliance perimeter walls.
 * Two additional targets are placed on the perimeter wall, one in front of each Tower Goal.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ultimategoal/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


public class RobotMovementIntegrated {
    private static final float TRACKING_BIAS = 0.5f;

    private static DcMotor motorBackLeft;
    private static DcMotor motorBackRight;
    private static DcMotor motorFrontLeft;
    private static DcMotor motorFrontRight;
    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

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
    //#region CONSTANTS
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    //#endregion

    //#region ROBOT_ATTRIBUTES
    // Attributes that must be modified to pertain to the robot
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
    static final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    static final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    static final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
    //#endregion

    //#region CLASS MEMBERS
    private static OpenGLMatrix lastLocation = null;
    private static VuforiaLocalizer vuforia = null;
    private static boolean targetVisible = false;
    private static float phoneXRotate    = 0;
    private static float phoneYRotate    = 0;
    private static float phoneZRotate    = 0;
    //#endregion

    void MoveRobotToLocation(float xFinal, float yFinal){
            int timesClose = 0;
            float distanceToMove = 0;

            while (timesClose > 3) {
                Point robotLocation = GetRobotLocation();
                float relativeX = (float)(xFinal - robotLocation.x);
                float relativeY = (float)(yFinal - robotLocation.y);

                if (Math.sqrt(Math.pow(relativeX, 2) + Math.pow(relativeY, 2)) < 0.5) {    //pick a better cutoff that we use to determine when the robot is close enough
                    distanceToMove = 0.1f;    //this is the distance we move if the robot is close
                    timesClose++;
                } else {
                    distanceToMove = 0.5f;    //this is the distance we move if the robot isn't close
                    timesClose = 0;
                }

                MoveRobot((float) Math.atan2(relativeY, relativeX), distanceToMove);
            }
        UtilHolonomic.stopWheelsSpeedMode();
    }

    static void MoveRobotToLocationDualSensor(float xFinal, float yFinal){
        int timesClose = 0;
        float distanceToMove = 0;
        Point robotEncoderLocation = null;

        while (timesClose > 3) {
            Point robotLocation = GetRobotLocation();
            if(robotLocation==null)
                continue;
            if(robotEncoderLocation==null)
                robotEncoderLocation = robotLocation;

            float relativeX = (float)(xFinal - robotLocation.x);
            float relativeY = (float)(yFinal - robotLocation.y);

            float angle = (float) Math.atan2(relativeY, relativeX);
            float angle2 = (float) Math.atan2(relativeY, relativeX);

            if (Math.sqrt(Math.pow(relativeX, 2) + Math.pow(relativeY, 2)) < 0.5) {    //pick a better cutoff that we use to determine when the robot is close enough
                distanceToMove = 0.1f;    //this is the distance we move if the robot is close
                timesClose++;
            } else {
                distanceToMove = 0.5f;    //this is the distance we move if the robot isn't close
                timesClose = 0;
            }
            //if vuforia and encoder are very off
            if(Math.sqrt(Math.pow(robotEncoderLocation.x - robotLocation.x, 2) + Math.pow(robotEncoderLocation.y - robotLocation.y, 2))>10){
                MoveRobot(angle2, distanceToMove);
                robotEncoderLocation.y+=Math.sin(angle2)*distanceToMove;
                robotEncoderLocation.x+=Math.cos(angle2)*distanceToMove;
            }else {
                MoveRobot(angle, distanceToMove);
                robotEncoderLocation.y=Math.sin(angle)*distanceToMove+(robotEncoderLocation.y+robotLocation.y)/2;
                robotEncoderLocation.x=Math.cos(angle)*distanceToMove+(robotEncoderLocation.x+robotLocation.x)/2;
            }
        }
        UtilHolonomic.stopWheelsSpeedMode();
    }

    static void MoveRobot(float angle, float distance) {
        UtilHolonomic.setWheelsToEncoderMode();
        double robotAngle = angle - Math.PI / 4;
        final double v1 = Math.cos(robotAngle);
        final double v2 = -Math.sin(robotAngle);
        final double v3 = Math.sin(robotAngle);
        final double v4 = -Math.cos(robotAngle);

        double FrontRight = Range.clip(v2, -1, 1);
        double FrontLeft = Range.clip(v1, -1, 1);
        double BackLeft = Range.clip(v3, -1, 1);
        double BackRight = Range.clip(v4, -1, 1);

        int moveAmount = (int) (distance * COUNTS_PER_INCH);

        int backLeftStartPosition = (int) (motorBackLeft.getCurrentPosition());
        int backRightStartPosition = (int) (motorBackRight.getCurrentPosition());
        int frontLeftStartPosition = (int) (motorFrontLeft.getCurrentPosition());
        int frontRightStartPosition = (int) (motorFrontRight.getCurrentPosition());

        int backLeftTargetPosition = (int) (motorBackLeft.getCurrentPosition() + Math.signum(BackLeft) * moveAmount);
        int backRightTargetPosition = (int) (motorBackRight.getCurrentPosition() + Math.signum(BackRight) * moveAmount);
        int frontLeftTargetPosition = (int) (motorFrontLeft.getCurrentPosition() + Math.signum(FrontLeft) * moveAmount);
        int frontRightTargetPosition = (int) (motorFrontRight.getCurrentPosition() + Math.signum(FrontRight) * moveAmount);


        motorBackLeft.setTargetPosition((int) backLeftTargetPosition);
        motorBackRight.setTargetPosition((int) backRightTargetPosition);
        motorFrontLeft.setTargetPosition((int) frontLeftTargetPosition);
        motorFrontRight.setTargetPosition((int) frontRightTargetPosition);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontRight.setPower(FrontRight);
        motorFrontLeft.setPower(FrontLeft);
        motorBackLeft.setPower(BackLeft);
        motorBackRight.setPower(BackRight);

        // for those motors that should be busy (power!=0) wait until they are done
        // reaching target position before returning from this function.

        double tolerance = TOLERANCE_WHEEL_POS;
        while (((((Math.abs(FrontRight)) > 0.01
                && Math.abs(motorFrontRight.getCurrentPosition() - frontRightTargetPosition) > tolerance))
                || (((Math.abs(FrontLeft)) > 0.01
                && Math.abs(motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition) > tolerance))
                || (((Math.abs(BackLeft)) > 0.01
                && Math.abs(motorBackLeft.getCurrentPosition() - backLeftTargetPosition) > tolerance))
                || (((Math.abs(BackRight)) > 0.01
                && Math.abs(motorBackRight.getCurrentPosition() - backRightTargetPosition) > tolerance)))) {

            // wait and check again until done running
            /*telemetry.addData("front right", "=%.2f %d %d %d %b", FrontRight, frontRightStartPosition,
                    motorFrontRight.getCurrentPosition(), frontRightTargetPosition,
                    ((Math.ceil(Math.abs(FrontRight)) > 0.0
                            && Math.abs(motorFrontRight.getCurrentPosition() - frontRightTargetPosition) > tolerance)));// ,
            // frontRightTargetPosition);
            telemetry.addData("front left", "=%.2f %d %d %d %b", FrontLeft,frontLeftStartPosition,
                    motorFrontLeft.getCurrentPosition(), frontLeftTargetPosition,
                    ((Math.ceil(Math.abs(FrontLeft)) > 0.0
                            && Math.abs(motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition) > tolerance)));// ,
            // frontLeftTargetPosition);
            telemetry.addData("back left", "=%.2f %d %d %d %b", BackLeft,backLeftStartPosition,
                    motorBackLeft.getCurrentPosition(),backLeftTargetPosition, ((Math.ceil(Math.abs(BackLeft)) > 0.0
                            && Math.abs(motorBackLeft.getCurrentPosition() - backLeftTargetPosition) > tolerance)));// ,
            // backLeftTargetPosition);
            telemetry.addData("back right", "=%.2f %d %d %d %b", BackRight,backRightStartPosition,
                    motorBackRight.getCurrentPosition(),backRightTargetPosition,
                    ((Math.ceil(Math.abs(BackRight)) > 0.0
                            && Math.abs(motorBackRight.getCurrentPosition() - backRightTargetPosition) > tolerance)));
            telemetry.update();*/
        }
    }

    private static Point GetRobotLocation(){
            System.out.println("---------------SIZE_OF_LIST=" + targetsUltimateGoal.size());

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    //telemetry.addData("Visible Target", trackable.getName());
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

            // Provide feedback as to where the robot is located (if we know).
        VectorF translation = null;
            if (targetVisible) {
                // express position (translation) of robot in inches.
                translation = lastLocation.getTranslation();
                //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        //translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                //telemetry.addData("Visible Target", "none");
            }
            //telemetry.update();

        // Disable Tracking when we are done;
        return new Point(translation.get(0), translation.get(1));
    }
    static VuforiaTrackables targetsUltimateGoal;
    static List<VuforiaTrackable> allTrackables;
    static void Deactivate(){
        targetsUltimateGoal.deactivate();
    }
    static void Initialize(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
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



        //VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        //frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);
        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
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

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsUltimateGoal.activate();
    }
}