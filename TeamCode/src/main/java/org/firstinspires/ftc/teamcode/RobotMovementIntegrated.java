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
import org.firstinspires.ftc.teamcode.AI.VuforiaInitializer;
import org.firstinspires.ftc.teamcode.Utilities.Point;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.AI.VuforiaInitializer.targetsUltimateGoal;
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

    private static DcMotor motorBackLeft;
    private static DcMotor motorBackRight;
    private static DcMotor motorFrontLeft;
    private static DcMotor motorFrontRight;

    static void MoveRobotToLocationRAW(float xInitial, float yInitial, float xFinal, float yFinal){
        MoveRobot((float)Math.atan2(yFinal-yInitial,xFinal-xInitial), (float)Math.sqrt(Math.pow(xFinal-xInitial, 2) + Math.pow(yFinal-yInitial, 2)));
        UtilHolonomic.stopWheelsSpeedMode();
    }

    static void MoveRobotToLocation(float xInitial, float yInitial, float xFinal, float yFinal){
        int timesClose = 0;
        float distanceToMove = 0;

        while (timesClose > 3) {
            Point robotLocation = null;
            if(xInitial ==-1 || yInitial == -1) {
                robotLocation =GetRobotLocation();
            }else{
                robotLocation = new Point(xInitial, yInitial);
            }
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
    static void MoveRobotToLocation(float xFinal, float yFinal){
        MoveRobotToLocation(-1, -1, xFinal, yFinal);
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
                robotEncoderLocation.y=(float)Math.sin(angle)*distanceToMove+(robotEncoderLocation.y+robotLocation.y)/2;
                robotEncoderLocation.x=(float)Math.cos(angle)*distanceToMove+(robotEncoderLocation.x+robotLocation.x)/2;
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
    private static boolean targetVisible = false;
    private static OpenGLMatrix lastLocation = null;
    public static Point GetRobotLocation(){
            System.out.println("---------------SIZE_OF_LIST=" + targetsUltimateGoal.size());

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : VuforiaInitializer.allTrackables) {
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
}