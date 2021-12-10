package org.firstinspires.ftc.teamcode.Utilities;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.os.Handler;
import android.os.Looper;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.Utilities.DeviceManager.motorBackLeft;
import static org.firstinspires.ftc.teamcode.Utilities.DeviceManager.motorBackRight;
import static org.firstinspires.ftc.teamcode.Utilities.DeviceManager.motorFrontLeft;
import static org.firstinspires.ftc.teamcode.Utilities.DeviceManager.motorFrontRight;

public class UtilHolonomic {

    public static final double DRIVE_MOTOR_POWER = 0.6;
    // goBILDA Yellow Jacket Planetary 13.7:1 (5202-0002-0014)
    public static final double COUNTS_PER_DRIVE_MOTOR_REV = 384.5; // counts per reevaluation of the motor
    public static final double SECONDSPERINCH = 2.0 / 24.0;
    public static final double TOLERANCE_WHEEL_POS = 100.0; //tolerance
    public static final double WHEEL_DIAMETER_INCHES = 3.937; // 100mm goBILDA mecanum
    public static final double INCHES_PER_ROTATION = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double DEG_PER_ROTATION = 100.0; // inches per rotation of 90mm traction wheel
    public static final double DISTANCE_FROM_WALL = 2.25;
    public static final int COUNTS_PER_INCH = (int) (COUNTS_PER_DRIVE_MOTOR_REV / INCHES_PER_ROTATION); // for 45deg
    // wheels
    public static final int COUNTS_PER_SQUARE = (int) (COUNTS_PER_INCH * 1); // for 45deg wheels
    public static final double CENTER_TO_WHEEL_DIST = COUNTS_PER_INCH * 8;//8 inches
    //AUTONOMOUS REFERENCES
    public static double MARKER_A_TO_PLATFORM_CENTER = 27;//towards the center of the platform
    public static double BRIDGE_TO_PLATFORM_CENTER = 51;//bridge to the center of the platform
    public static double EDGE_TO_PLATFORM = 31;//distance from the edge to the platform
    public static double ROBOT_WALL_CLEARANCE = 5;//clearance to put the robot from the edge
    public static double WALL_ROBOT_TO_EDGE_LOAD = 47;//distance from the robot to the edge with slipping and a load
    public static double EDGE_TO_PLATFORM_CLEARANCE = EDGE_TO_PLATFORM + 4 - ROBOT_WALL_CLEARANCE;//distance from the robot to the clearance distance
    public static double WALL_TO_CENTER = 29;//distance from the robot to the clearance distance

    public static double AUTONOMOUS_SPEED = 1;//speed for the autonomous programs

    public static double LOAD_CONST = 1.53333333;
    public static DistanceSensor sensorDistance;

    //private static Servo platform_left;
    // private static Servo platform_right;
    static int relativeLayoutId;
    static View relativeLayout;
    private static DcMotor motorRight;
    private static DcMotor motorLeft;
    private static Telemetry telemetry;
    // initialize these in InitExtraSensors if using
    private static ColorSensor colorSensor;
    private static float hsvValues[] = {0F, 0F, 0F};
    private static float values[];
    final double SCALE_FACTOR = 255;

    //#region Initialization
    public UtilHolonomic(DcMotor frontRightMotor, DcMotor frontLeftMotor, DcMotor backRightMotor, DcMotor backLeftMotor,
                         Telemetry telemetryIn) {

        motorBackLeft = backLeftMotor;
        motorBackRight = backRightMotor;
        motorFrontLeft = frontLeftMotor;
        motorFrontRight = frontRightMotor;

        telemetry = telemetryIn;

    }

    ///

    public static double Measure_ONLOAD(double value) {
        return LOAD_CONST * value;
    }

    public static void InitPlatform(HardwareMap hardwareMap) {
        //  platform_left = hardwareMap.servo.get("platform_left");
        //  platform_right = hardwareMap.servo.get("platform_right");
        GrabPlaform(true);
    }

    //not being used in 2020 config
    public static void InitExtraSensors(HardwareMap hardwareMap) {
        // get a reference to the color sensor.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        colorSensor.enableLed(false);
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        //   telemetry.addData("Distance (cm)",
        //         String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        // hsvValues is an array that will hold the hue, saturation, and value
        // information.
        hsvValues = new float[]{0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        float values[] = hsvValues;
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB
        // sensor.
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id",
                hardwareMap.appContext.getPackageName());
        View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    }
    //#endregion

    //#endregion

    //#region MotorUnilities
    public static void setWheelsToEncoderMode() {
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetEncoderOnMotors();
    }

    public static void setWheelsToSpeedMode() {

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void stopWheelsSpeedMode() {

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
    }

    public static void resetEncoderOnMotors() {
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //#endregion

    public static void GrabPlaform(boolean open) {
        if (open) {
            // platform_right.setPosition(1);
            //  platform_left.setPosition(0);
        } else {
            //  platform_right.setPosition(0);
            //  platform_left.setPosition(1);
        }
    }


    //#region Driving
    public static void DriveByDistanceByTime(double x, double y, double distance, String unit) {// inches
        setWheelsToEncoderMode();
        double r = Math.hypot((-x), (-y));
        double robotAngle = Math.atan2((-y), (-x)) - Math.PI / 4;
        final double v1 = r * Math.cos(robotAngle);
        final double v2 = -r * Math.sin(robotAngle);
        final double v3 = r * Math.sin(robotAngle);
        final double v4 = -r * Math.cos(robotAngle);

        double FrontRight = Range.clip(v2, -1, 1);
        double FrontLeft = Range.clip(v1, -1, 1);
        double BackLeft = Range.clip(v3, -1, 1);
        double BackRight = Range.clip(v4, -1, 1);

        //int moveAmount = (int) (distance * COUNTS_PER_INCH);

        double distance_time = distance * SECONDSPERINCH;


        ElapsedTime et = new ElapsedTime();
        et.reset();

        while (et.seconds() < distance_time) {
            motorFrontRight.setPower(FrontRight);
            motorFrontLeft.setPower(FrontLeft);
            motorBackLeft.setPower(BackLeft);
            motorBackRight.setPower(BackRight);
        }
        et.reset();
        stopWheelsSpeedMode();

    }

    public static void DriveByDistance(double x, double y, double distance) {
        DriveByDistance(x, y, distance, -1);
    }

    //#region Driving
    public static void DriveByDistance(double x, double y, double distance, double sensor_distance) {// inches
        setWheelsToEncoderMode();
        double r = Math.hypot((-x), (-y));
        double robotAngle = Math.atan2((-y), (-x)) - Math.PI / 4;
        final double v1 = r * Math.cos(robotAngle);
        final double v2 = -r * Math.sin(robotAngle);
        final double v3 = r * Math.sin(robotAngle);
        final double v4 = -r * Math.cos(robotAngle);

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
            if (sensor_distance != -1 && !(Double.isNaN(sensorDistance.getDistance(DistanceUnit.INCH))) && sensorDistance.getDistance(DistanceUnit.INCH) < DISTANCE_FROM_WALL) {
                telemetry.addData("DATA", "Sensor Triggered");
                telemetry.update();
                break;
            }
            // wait and check again until done running
            telemetry.addData("front right", "=%.2f %d %d %d %b", FrontRight, frontRightStartPosition,
                    motorFrontRight.getCurrentPosition(), frontRightTargetPosition,
                    ((Math.ceil(Math.abs(FrontRight)) > 0.0
                            && Math.abs(motorFrontRight.getCurrentPosition() - frontRightTargetPosition) > tolerance)));// ,
            // frontRightTargetPosition);
            telemetry.addData("front left", "=%.2f %d %d %d %b", FrontLeft, frontLeftStartPosition,
                    motorFrontLeft.getCurrentPosition(), frontLeftTargetPosition,
                    ((Math.ceil(Math.abs(FrontLeft)) > 0.0
                            && Math.abs(motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition) > tolerance)));// ,
            // frontLeftTargetPosition);
            telemetry.addData("back left", "=%.2f %d %d %d %b", BackLeft, backLeftStartPosition,
                    motorBackLeft.getCurrentPosition(), backLeftTargetPosition, ((Math.ceil(Math.abs(BackLeft)) > 0.0
                            && Math.abs(motorBackLeft.getCurrentPosition() - backLeftTargetPosition) > tolerance)));// ,
            // backLeftTargetPosition);
            telemetry.addData("back right", "=%.2f %d %d %d %b", BackRight, backRightStartPosition,
                    motorBackRight.getCurrentPosition(), backRightTargetPosition,
                    ((Math.ceil(Math.abs(BackRight)) > 0.0
                            && Math.abs(motorBackRight.getCurrentPosition() - backRightTargetPosition) > tolerance)));
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        stopWheelsSpeedMode();

    }

    public static void driveUntilSensor(double x, double distance) {// inches
        double y = 0;
        setWheelsToEncoderMode();
        double r = Math.hypot((-x), (-y));
        double robotAngle = Math.atan2((-y), (-x)) - Math.PI / 4;
        final double v1 = r * Math.cos(robotAngle);
        final double v2 = -r * Math.sin(robotAngle);
        final double v3 = r * Math.sin(robotAngle);
        final double v4 = -r * Math.cos(robotAngle);

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

        setWheelsToSpeedMode();

        motorFrontRight.setPower(FrontRight);
        motorFrontLeft.setPower(FrontLeft);
        motorBackLeft.setPower(BackLeft);
        motorBackRight.setPower(BackRight);

        // for those motors that should be busy (power!=0) wait until they are done
        // reaching target position before returning from this function.

        while (sensorDistance.getDistance(DistanceUnit.INCH) > distance) {
            try {
                Thread.sleep(500);

            } catch (Exception e) {

            }
        }
        stopWheelsSpeedMode();

    }


    public static void DriveByDistAndRot(double x, double y, double rotation, double distance, String unit) {// inches
        setWheelsToEncoderMode();
        double r = Math.hypot((-x), (-y));
        double robotAngle = Math.atan2((-y), (-x)) - Math.PI / 4;

        double radians = rotation * 3.1415 / 180;
        double distanceRot = radians * CENTER_TO_WHEEL_DIST;

        final double v1 = r * Math.cos(robotAngle);
        final double v2 = -r * Math.sin(robotAngle);
        final double v3 = r * Math.sin(robotAngle);
        final double v4 = -r * Math.cos(robotAngle);

        double FrontRight = Range.clip(v2, -1, 1);
        double FrontLeft = Range.clip(v1, -1, 1);
        double BackLeft = Range.clip(v3, -1, 1);
        double BackRight = Range.clip(v4, -1, 1);

        telemetry.addData("UtilHolonomic", "FRONTRIGHT=" + FrontRight);
        telemetry.addData("UtilHolonomic", "FRONTLEFT=" + FrontLeft);
        telemetry.addData("UtilHolonomic", "BACKLEFT=" + BackLeft);
        telemetry.addData("UtilHolonomic", "BACKRIGH=" + BackRight);
        telemetry.update();
        int moveAmount = (int) (distance * COUNTS_PER_INCH);
        // if(unit.equals("inch")) {
        // moveAmount = (int) (distance * COUNTS_PER_INCH);
        // }else
        // if(unit.equals("square")) {
        // moveAmount = (int) (distance * COUNTS_PER_SQUARE);
        // }
        int backLeftTargetPosition = (int) (motorBackLeft.getCurrentPosition() + Math.signum(BackLeft) * moveAmount + distanceRot);
        int backRightTargetPosition = (int) (motorBackRight.getCurrentPosition() + Math.signum(BackRight) * moveAmount + distanceRot);
        int frontLeftTargetPosition = (int) (motorFrontLeft.getCurrentPosition() + Math.signum(FrontLeft) * moveAmount + distanceRot);
        int frontRightTargetPosition = (int) (motorFrontRight.getCurrentPosition()
                + Math.signum(FrontRight) * moveAmount + distanceRot);


        motorBackLeft.setTargetPosition((int) backLeftTargetPosition);
        motorBackRight.setTargetPosition((int) backRightTargetPosition);
        motorFrontLeft.setTargetPosition((int) frontLeftTargetPosition);
        motorFrontRight.setTargetPosition((int) frontRightTargetPosition);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int maxTargetPosition = 0;
        int[] tpArray = new int[]{backLeftTargetPosition, backRightTargetPosition, frontLeftTargetPosition, frontRightTargetPosition};
        for (int i = 0; i < 4; i++) {
            if (maxTargetPosition < Math.abs(tpArray[i])) {
                maxTargetPosition = Math.abs(tpArray[i]);
            }
        }

        double mfr = Math.signum(FrontRight) * (frontRightTargetPosition / maxTargetPosition);
        double mfl = Math.signum(FrontLeft) * (frontLeftTargetPosition / maxTargetPosition);
        double mbl = Math.signum(BackLeft) * (backLeftTargetPosition / maxTargetPosition);
        double mbr = Math.signum(BackRight) * (backRightTargetPosition / maxTargetPosition);
        motorFrontRight.setPower(mfr);
        motorFrontLeft.setPower(mfl);
        motorBackLeft.setPower(mbl);
        motorBackRight.setPower(mbr);
        telemetry.addData("[setPower]:", "mfr=%d", mfr);
        telemetry.addData("[setPower]:", "mfl=%d", mfl);
        telemetry.addData("[setPower]:", "mbl=%d", mbl);
        telemetry.addData("[setPower]:", "mbr=%d", mbr);
        // for those motors that should be busy (power!=0) wait until they are done
        // reaching target position before returning from this function.

        double tolerance = TOLERANCE_WHEEL_POS;
        while ((((Math.abs(FrontRight)) > 0.01
                && Math.abs(motorFrontRight.getCurrentPosition() - frontRightTargetPosition) > tolerance))
                || (((Math.abs(FrontLeft)) > 0.01
                && Math.abs(motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition) > tolerance))
                || (((Math.abs(BackLeft)) > 0.01
                && Math.abs(motorBackLeft.getCurrentPosition() - backLeftTargetPosition) > tolerance))
                || (((Math.abs(BackRight)) > 0.01
                && Math.abs(motorBackRight.getCurrentPosition() - backRightTargetPosition) > tolerance))) {
            // wait and check again until done running
            telemetry.addData("front right", "=%.2f  %d %b", FrontRight,
                    motorFrontRight.getCurrentPosition() - frontRightTargetPosition,
                    ((Math.ceil(Math.abs(FrontRight)) > 0.0
                            && Math.abs(motorFrontRight.getCurrentPosition() - frontRightTargetPosition) > tolerance)));// ,
            // frontRightTargetPosition);
            telemetry.addData("front left", "=%.2f %d %b", FrontLeft,
                    motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition,
                    ((Math.ceil(Math.abs(FrontLeft)) > 0.0
                            && Math.abs(motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition) > tolerance)));// ,
            // frontLeftTargetPosition);
            telemetry.addData("back left", "=%.2f %d %b", BackLeft,
                    motorBackLeft.getCurrentPosition() - backLeftTargetPosition, ((Math.ceil(Math.abs(BackLeft)) > 0.0
                            && Math.abs(motorBackLeft.getCurrentPosition() - backLeftTargetPosition) > tolerance)));// ,
            // backLeftTargetPosition);
            telemetry.addData("back right", "=%.2f %d %b", BackRight,
                    motorBackRight.getCurrentPosition() - backRightTargetPosition,
                    ((Math.ceil(Math.abs(BackRight)) > 0.0
                            && Math.abs(motorBackRight.getCurrentPosition() - backRightTargetPosition) > tolerance)));
            telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        stopWheelsSpeedMode();

    }

    public static void DriveBySpeed(double x, double y, double rotation) {// inches

        double r = Math.hypot((-x), (-y));
        double robotAngle = Math.atan2((-y), (-x)) - Math.PI / 4;
        double rightX = rotation;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = -r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = -r * Math.cos(robotAngle) - rightX;

        double FrontRight = Range.clip(v2, -1, 1);
        double FrontLeft = Range.clip(v1, -1, 1);
        double BackLeft = Range.clip(v3, -1, 1);
        double BackRight = Range.clip(v4, -1, 1);


        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setPower(FrontRight);
        motorFrontLeft.setPower(FrontLeft);
        motorBackLeft.setPower(BackLeft);
        motorBackRight.setPower(BackRight);
    }

    @SuppressLint("NewApi")
    public static void DriveUntilColor(double x, double y, double rotation, double distance, String unit) {// inches
        setWheelsToEncoderMode();
        double r = Math.hypot((-x), (-y));
        double robotAngle = Math.atan2((-y), (-x)) - Math.PI / 4;
        double rightX = rotation;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = -r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = -r * Math.cos(robotAngle) - rightX;

        double FrontRight = Range.clip(v2, -1, 1);
        double FrontLeft = Range.clip(v1, -1, 1);
        double BackLeft = Range.clip(v3, -1, 1);
        double BackRight = Range.clip(v4, -1, 1);

        double moveAmount = distance;
        if (unit.equals("inch")) {
            moveAmount = (int) (distance * COUNTS_PER_INCH);
        } else if (unit.equals("square")) {
            moveAmount = (int) (distance * COUNTS_PER_SQUARE);
        }
        int backLeftTargetPosition = (int) (motorBackLeft.getCurrentPosition() + Math.signum(BackLeft) * moveAmount);
        int backRightTargetPosition = (int) (motorBackRight.getCurrentPosition() + Math.signum(BackRight) * moveAmount);
        int frontLeftTargetPosition = (int) (motorFrontLeft.getCurrentPosition() + Math.signum(FrontLeft) * moveAmount);
        int frontRightTargetPosition = (int) (motorFrontRight.getCurrentPosition()
                + Math.signum(FrontRight) * moveAmount);

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

        //colorSensor.enableLed(true);

        // for those motors that should be busy (power!=0) wait until they are done
        // reaching target position before returning from this function.


        //      Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR), (int) (colorSensor.green() * SCALE_FACTOR),
        //    (int) (colorSensor.blue() * SCALE_FACTOR), hsvValues);

        /* send the info back to driver station using telemetry function.
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();
        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        boolean foundBlue = false;
        boolean foundRed = false;

        if (hsvValues[0] > 200 && hsvValues[0] < 250) {
            relativeLayout.setBackgroundColor(Color.BLUE);
            foundBlue = true;
            telemetry.addData("Blue", hsvValues[0]);
            telemetry.update();
        } else {
            // look for the hue in the red range
            if (hsvValues[0] < 10 || hsvValues[0] > 330) {
                relativeLayout.setBackgroundColor(Color.RED);
                foundRed = true;
                telemetry.addData("Blue", hsvValues[0]);
                telemetry.update();
            }
        }

        // updates needed here to drive until foundRed or foundBlue;

        double tolerance = TOLERANCE_WHEEL_POS;
        while ((((Math.abs(FrontRight)) > 0.01
                && Math.abs(motorFrontRight.getCurrentPosition() - frontRightTargetPosition) > tolerance))
                || (((Math.abs(FrontLeft)) > 0.01
                        && Math.abs(motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition) > tolerance))
                || (((Math.abs(BackLeft)) > 0.01
                        && Math.abs(motorBackLeft.getCurrentPosition() - backLeftTargetPosition) > tolerance))
                || (((Math.abs(BackRight)) > 0.01
                        && Math.abs(motorBackRight.getCurrentPosition() - backRightTargetPosition) > tolerance))
                || (!foundBlue || !foundRed)) {


            // wait and check again until done running
            // telemetry.addData("front right", "=%.2f %d %b", FrontRight,
            // motorFrontRight.getCurrentPosition() -
            // frontRightTargetPosition,((Math.ceil(Math.abs(FrontRight)) > 0.0 &&
            // Math.abs(motorFrontRight.getCurrentPosition() - frontRightTargetPosition) >
            // tolerance)));//, frontRightTargetPosition);
            // telemetry.addData("front left", "=%.2f %d %b", FrontLeft,
            // motorFrontLeft.getCurrentPosition() -
            // frontLeftTargetPosition,((Math.ceil(Math.abs(FrontLeft)) > 0.0 &&
            // Math.abs(motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition) >
            // tolerance)));//, frontLeftTargetPosition);
            // telemetry.addData("back left", "=%.2f %d %b", BackLeft,
            // motorBackLeft.getCurrentPosition() - backLeftTargetPosition,
            // ((Math.ceil(Math.abs(BackLeft)) > 0.0 &&
            // Math.abs(motorBackLeft.getCurrentPosition() - backLeftTargetPosition) >
            // tolerance)));//, backLeftTargetPosition);
            // telemetry.addData("back right", "=%.2f %d %b", BackRight,
            // motorBackRight.getCurrentPosition() - backRightTargetPosition,
            // ((Math.ceil(Math.abs(BackRight)) > 0.0 &&
            // Math.abs(motorBackRight.getCurrentPosition() - backRightTargetPosition) >
            // tolerance)));
            // telemetry.update();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);

        colorSensor.enableLed(false);
        */
    }
    //#endregion

    //#region Other Utilities
    public static void MoveClaw(final DcMotor motor, double position) {

        Thread thr = new Thread(new Runnable() {
            @Override
            public void run() {
                // while() //while the motor is not at the position specified which would either
                // be the top position or 0(Where it starts at the bottor)
                new Handler(Looper.getMainLooper()).post(new Runnable() {
                    @Override
                    public void run() {

                    }
                });
            }

        });
        thr.start();
    }

    public static void TwoWheelDrive(double leftInput, double rightInput, int mode) {
        double rightDrive = UtilMain.scaleInput(rightInput, mode);
        double leftDrive = UtilMain.scaleInput(-leftInput, mode);

        double finalRight = Range.clip(rightDrive, -1, 1);
        double finalLeft = Range.clip(leftDrive, -1, 1);

        // write the values to the motors
        motorLeft.setPower(finalLeft);
        motorRight.setPower(finalRight);
    }

    public static void DriveFixedDistance(double directionDrive, double inches, boolean isFast) {
        motorRight.setMode(STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setMode(STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double rotations = inches / INCHES_PER_ROTATION;
        int targetPositionR = (int) (motorRight.getCurrentPosition()
                + (directionDrive * rotations * COUNTS_PER_DRIVE_MOTOR_REV));
        int targetPositionL = (int) (motorLeft.getCurrentPosition()
                + (directionDrive * rotations * COUNTS_PER_DRIVE_MOTOR_REV));
        telemetry.addData("Drive Position R", "= %d  %d  %d", motorRight.getCurrentPosition(),
                motorRight.getTargetPosition(), targetPositionR);
        telemetry.addData("Drive Position L", "= %d  %d  %d", motorLeft.getCurrentPosition(),
                motorLeft.getTargetPosition(), targetPositionL);
        telemetry.update();
        while ((/* touchSensor.getState() == false && */
                Math.abs(motorRight.getCurrentPosition() - (targetPositionR)) > TOLERANCE_WHEEL_POS)
                || Math.abs(motorLeft.getCurrentPosition() - (targetPositionL)) > TOLERANCE_WHEEL_POS) {
            motorRight.setTargetPosition(
                    (int) (motorRight.getCurrentPosition() + (int) directionDrive * TOLERANCE_WHEEL_POS));
            if (isFast) {
                motorRight.setPower(DRIVE_MOTOR_POWER * 2);
            } else {
                motorRight.setPower(DRIVE_MOTOR_POWER);
            }
            motorLeft.setTargetPosition(
                    (int) (motorRight.getCurrentPosition() + (int) directionDrive * TOLERANCE_WHEEL_POS));
            if (isFast) {
                motorLeft.setPower(DRIVE_MOTOR_POWER * 2);
            } else {
                motorLeft.setPower(DRIVE_MOTOR_POWER);
            }
            telemetry.addData("Drive Position R", "= %d  %d  %d", motorRight.getCurrentPosition(),
                    motorRight.getTargetPosition(), targetPositionR);
            telemetry.addData("Drive Position L", "= %d  %d  %d", motorLeft.getCurrentPosition(),
                    motorLeft.getTargetPosition(), targetPositionL);
            telemetry.update();

        }
        motorRight.setPower(0.0);
        motorRight.setMode(RUN_WITHOUT_ENCODER);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft.setPower(0.0);
        motorLeft.setMode(RUN_WITHOUT_ENCODER);
    }

    public static void DriveAngledDistance(double directionDrive, double inches, boolean isFast) {
        double WEIGHT_R = 1.2;
        motorRight.setMode(STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setMode(STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double rotations = inches / INCHES_PER_ROTATION;
        int targetPositionR = (int) (motorRight.getCurrentPosition()
                + (directionDrive * WEIGHT_R * rotations * COUNTS_PER_DRIVE_MOTOR_REV));
        int targetPositionL = (int) (motorLeft.getCurrentPosition()
                + (directionDrive * rotations * COUNTS_PER_DRIVE_MOTOR_REV));
        telemetry.addData("Drive Position R", "= %d  %d  %d", motorRight.getCurrentPosition(),
                motorRight.getTargetPosition(), targetPositionR);
        telemetry.addData("Drive Position L", "= %d  %d  %d", motorLeft.getCurrentPosition(),
                motorLeft.getTargetPosition(), targetPositionL);
        telemetry.update();
        while ((/* touchSensor.getState() == false && */
                Math.abs(motorRight.getCurrentPosition() - (targetPositionR)) > TOLERANCE_WHEEL_POS)
                || Math.abs(motorLeft.getCurrentPosition() - (targetPositionL)) > TOLERANCE_WHEEL_POS) {
            motorRight.setTargetPosition(
                    (int) (motorRight.getCurrentPosition() + (int) directionDrive * TOLERANCE_WHEEL_POS));
            if (isFast) {
                motorRight.setPower(WEIGHT_R * DRIVE_MOTOR_POWER * 2);
            } else {
                motorRight.setPower(WEIGHT_R * DRIVE_MOTOR_POWER);
            }
            motorLeft.setTargetPosition(
                    (int) (motorRight.getCurrentPosition() + (int) directionDrive * TOLERANCE_WHEEL_POS));
            if (isFast) {
                motorLeft.setPower(DRIVE_MOTOR_POWER * 2);
            } else {
                motorLeft.setPower(DRIVE_MOTOR_POWER);
            }
            telemetry.addData("Drive Position R", "= %d  %d  %d", motorRight.getCurrentPosition(),
                    motorRight.getTargetPosition(), targetPositionR);
            telemetry.addData("Drive Position L", "= %d  %d  %d", motorLeft.getCurrentPosition(),
                    motorLeft.getTargetPosition(), targetPositionL);
            telemetry.update();

        }
        motorRight.setPower(0.0);
        motorRight.setMode(RUN_WITHOUT_ENCODER);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft.setPower(0.0);
        motorLeft.setMode(RUN_WITHOUT_ENCODER);
    }

    public static void DriveFixedDegrees(double directionDrive, double degrees) {
        motorRight.setMode(STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double rotations = degrees / DEG_PER_ROTATION;
        int targetPositionR = (int) (motorRight.getCurrentPosition()
                + (directionDrive * rotations * COUNTS_PER_DRIVE_MOTOR_REV));
        int targetPositionL = (int) (motorLeft.getCurrentPosition()
                + (directionDrive * rotations * COUNTS_PER_DRIVE_MOTOR_REV));
        telemetry.addData("Drive Position R", "= %d  %d  %d", motorRight.getCurrentPosition(),
                motorRight.getTargetPosition(), targetPositionR);
        telemetry.addData("Drive Position L", "= %d  %d  %d", motorLeft.getCurrentPosition(),
                motorLeft.getTargetPosition(), targetPositionL);
        telemetry.update();
        while ((/* touchSensor.getState() == false && */
                Math.abs(motorRight.getCurrentPosition() - (targetPositionR)) > TOLERANCE_WHEEL_POS)
                || Math.abs(motorLeft.getCurrentPosition() - (targetPositionL)) > TOLERANCE_WHEEL_POS) {
            motorRight.setTargetPosition(
                    (int) (motorRight.getCurrentPosition() + (int) directionDrive * TOLERANCE_WHEEL_POS));
            motorRight.setPower(DRIVE_MOTOR_POWER);
            motorLeft.setTargetPosition(
                    (int) (motorRight.getCurrentPosition() + (int) directionDrive * TOLERANCE_WHEEL_POS));
            motorLeft.setPower(DRIVE_MOTOR_POWER);
            telemetry.addData("Drive Position R", "= %d  %d  %d", motorRight.getCurrentPosition(),
                    motorRight.getTargetPosition(), targetPositionR);
            telemetry.addData("Drive Position L", "= %d  %d  %d", motorLeft.getCurrentPosition(),
                    motorLeft.getTargetPosition(), targetPositionL);
            telemetry.update();

        }
        motorRight.setPower(0.0);
        motorRight.setMode(RUN_WITHOUT_ENCODER);
        motorLeft.setPower(0.0);
        motorLeft.setMode(RUN_WITHOUT_ENCODER);
    }
}
