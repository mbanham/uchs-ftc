package org.firstinspires.ftc.team11288;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.graphics.Color;
import android.os.Handler;
import android.os.Looper;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.text.DateFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ThreadFactory;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;


public class Util {
    private DcMotor motorRight;
    private DcMotor motorLeft;
    private DcMotor liftMotor;
    private DcMotor armMotor;
    private Telemetry telemetry;
    private DigitalChannel touchSensor;
    private DigitalChannel liftSensor;
    //vuforia
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String STONE = "Stone";
    public static final String SKYSTONE = "Skystone";
    private static final String VUFORIA_KEY = "ASVozkX/////AAAAGX+Aimqfn0YRqafZGVaD2MIhBsmxiHLTd4r2XyoV4F/VEvRMnL1mLn7NDtl1onYGhHmJADQR8nt0aX4rZLIAb/7+XxI7LLZV4X0tMBDQyBL6IWcEdgMD63hTKncdP8NsIVJxJOY971/5pVdU50XisgiiAhq3b6D9twKLfGZ9EI2M4XXM0B7BxdA7x7YMD5QcMDf96myKGsPhVlkwz8XvBdbnOvZZg2FoxmhqExRp33AKii1GZRDwvfeco0hEOKusdwOkjbJ5RTJ+9T3fAysvqSovSG8iAWZ98qrG2xop2gK73UPJaY4vj5/1yVBKFMnWt42P931ybmEW1/c5dc8LR1CyD8jCxlgqypf9oCz/q89j";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private final double LIFT_MOTOR_POWER = 0.65;
    private final double ARM_MOTOR_POWER = 0.15;
    private final double DRIVE_MOTOR_POWER = 0.75;
    static final double     COUNTS_PER_MOTOR_REV    = 1250.0; //HD Hex Motor (REV-41-1301) 40:1
    static final double     COUNTS_PER_DRIVE_MOTOR_REV    = 300; // counts per reevaluation of the motor
    static final double INCREMENT_MOTOR_MOVE = 175.0; // move set amount at a time
    static final double INCREMENT_DRIVE_MOTOR_MOVE = 30.0; // move set amount at a time
    static final double INCHES_PER_ROTATION = 11.137; //inches per rotation of 90mm traction wheel
    static final double DEG_PER_ROTATION = 100.0; //inches per rotation of 90mm traction wheel
    static final double claw_arm_max_distance = 200;

    //2019 Code changes
    private DcMotor  motorBackLeft;
    private DcMotor  motorBackRight;
    private DcMotor  motorFrontLeft;
    private DcMotor  motorFrontRight;

    static final int COUNTS_PER_INCH= (int) ((1.4142 * (COUNTS_PER_DRIVE_MOTOR_REV)) / (4.0 * Math.PI)); //for 45deg wheels
    static final int COUNTS_PER_SQUARE = (int) (COUNTS_PER_INCH * 1); //for 45deg wheels

    //initialize these in InitExtraSensors if using
    private ColorSensor colorSensor;
    float hsvValues[] = {0F, 0F, 0F};
    float values[];
    final double SCALE_FACTOR = 255;
    int relativeLayoutId;
    static View relativeLayout;


    ///

    public Util(DcMotor frontRightMotor, DcMotor frontLeftMotor, DcMotor backRightMotor, DcMotor backLeftMotor,
                Telemetry telemetryIn) {

        motorBackLeft=backLeftMotor;
        motorBackRight=backRightMotor;
        motorFrontLeft=frontLeftMotor;
        motorFrontRight=frontRightMotor;

        telemetry=telemetryIn;
//        motorLeft = leftMotor;
//        motorRight = rightMotor;
//        liftMotor = liftMotorIn;
//        armMotor = armMotorIn;
//        telemetry = telemetryIn;
//        touchSensor = touchSensorIn;
//       // liftSensor = liftSensorIn;
    }

    public void InitExtraSensors(HardwareMap hardwareMap){
        // get a reference to the color sensor.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        // hsvValues is an array that will hold the hue, saturation, and value information.
        hsvValues = new float[] {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        float values[] = hsvValues;
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    }
    public void InitVuforia(HardwareMap hwm){
        initVuforiaRaw();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hwm);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

    }

    private void initVuforiaRaw() {
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
    private void initTfod(HardwareMap hwm) {
        int tfodMonitorViewId = hwm.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwm.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE, SKYSTONE);
    }

    public List<Recognition> GetObjectsInFrame(){
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        return updatedRecognitions;
    }

    //Routines for 2019-2020 - based on TeleopDrive code from
    // 2017

    public void drivebyDistance(double x, double y, double rotation, double distance, String unit) {//inches
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


        int moveAmount = (int) (distance * COUNTS_PER_INCH);
//        if(unit.equals("inch")) {
//            moveAmount = (int) (distance * COUNTS_PER_INCH);
//        }else
//        if(unit.equals("square")) {
//            moveAmount = (int) (distance * COUNTS_PER_SQUARE);
//        }
        int backLeftTargetPosition = (int) (motorBackLeft.getCurrentPosition() + Math.signum(BackLeft)* moveAmount);
        int backRightTargetPosition = (int) (motorBackRight.getCurrentPosition() + Math.signum(BackRight)* moveAmount);
        int frontLeftTargetPosition = (int) (motorFrontLeft.getCurrentPosition() + Math.signum(FrontLeft)* moveAmount);
        int frontRightTargetPosition = (int) (motorFrontRight.getCurrentPosition() + Math.signum(FrontRight)* moveAmount);

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

        //for those motors that should be busy (power!=0) wait until they are done
        //reaching target position before returning from this function.

        double tolerance = INCREMENT_DRIVE_MOTOR_MOVE;
        while ((((Math.abs(FrontRight)) > 0.01 && Math.abs(motorFrontRight.getCurrentPosition() - frontRightTargetPosition) > tolerance)) ||
                (((Math.abs(FrontLeft)) > 0.01 && Math.abs(motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition) > tolerance)) ||
                (((Math.abs(BackLeft)) > 0.01 && Math.abs(motorBackLeft.getCurrentPosition() - backLeftTargetPosition) > tolerance)) ||
                (((Math.abs(BackRight)) > 0.01 && Math.abs(motorBackRight.getCurrentPosition() - backRightTargetPosition) > tolerance))){
            //wait and check again until done running
            telemetry.addData("front right", "=%.2f  %d %b", FrontRight, motorFrontRight.getCurrentPosition() - frontRightTargetPosition,((Math.ceil(Math.abs(FrontRight)) > 0.0 && Math.abs(motorFrontRight.getCurrentPosition() - frontRightTargetPosition) > tolerance)));//, frontRightTargetPosition);
            telemetry.addData("front left", "=%.2f %d %b", FrontLeft, motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition,((Math.ceil(Math.abs(FrontLeft)) > 0.0 && Math.abs(motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition) > tolerance)));//, frontLeftTargetPosition);
            telemetry.addData("back left", "=%.2f %d %b",  BackLeft, motorBackLeft.getCurrentPosition() - backLeftTargetPosition, ((Math.ceil(Math.abs(BackLeft)) > 0.0 && Math.abs(motorBackLeft.getCurrentPosition() - backLeftTargetPosition) > tolerance)));//, backLeftTargetPosition);
            telemetry.addData("back right", "=%.2f %d %b", BackRight, motorBackRight.getCurrentPosition() - backRightTargetPosition, ((Math.ceil(Math.abs(BackRight)) > 0.0 && Math.abs(motorBackRight.getCurrentPosition() - backRightTargetPosition) > tolerance)));
            telemetry.update();
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

    }
    @SuppressLint("NewApi")
    public void driveUntilColor(double x, double y, double rotation, double distance, String unit) {//inches
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
        if(unit.equals("inch")) {
            moveAmount = (int) (distance * COUNTS_PER_INCH);
        }else
        if(unit.equals("square")) {
            moveAmount = (int) (distance * COUNTS_PER_SQUARE);
        }
        int backLeftTargetPosition = (int) (motorBackLeft.getCurrentPosition() + Math.signum(BackLeft)* moveAmount);
        int backRightTargetPosition = (int) (motorBackRight.getCurrentPosition() + Math.signum(BackRight)* moveAmount);
        int frontLeftTargetPosition = (int) (motorFrontLeft.getCurrentPosition() + Math.signum(FrontLeft)* moveAmount);
        int frontRightTargetPosition = (int) (motorFrontRight.getCurrentPosition() + Math.signum(FrontRight)* moveAmount);

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



        colorSensor.enableLed(true);
        //for those motors that should be busy (power!=0) wait until they are done
        //reaching target position before returning from this function.

        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();
        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        boolean foundBlue=false;
        boolean foundRed=false;

        if(hsvValues[0]>200 && hsvValues[0]<250) {
            relativeLayout.setBackgroundColor(Color.BLUE);
            foundBlue=true;
        }
        else{
            //look for the hue in the red range
            if(hsvValues[0]<10 || hsvValues[0]>330) {
                relativeLayout.setBackgroundColor(Color.RED);
                foundRed=true;
            }
        }



    //updates needed here to drive until foundRed or foundBlue;

        double tolerance = INCREMENT_DRIVE_MOTOR_MOVE;
        while ((((Math.abs(FrontRight)) > 0.01 && Math.abs(motorFrontRight.getCurrentPosition() - frontRightTargetPosition) > tolerance)) ||
                (((Math.abs(FrontLeft)) > 0.01 && Math.abs(motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition) > tolerance)) ||
                (((Math.abs(BackLeft)) > 0.01 && Math.abs(motorBackLeft.getCurrentPosition() - backLeftTargetPosition) > tolerance)) ||
                (((Math.abs(BackRight)) > 0.01 && Math.abs(motorBackRight.getCurrentPosition() - backRightTargetPosition) > tolerance)) ||
                (!foundBlue || !foundRed)){
            //wait and check again until done running
//            telemetry.addData("front right", "=%.2f  %d %b", FrontRight, motorFrontRight.getCurrentPosition() - frontRightTargetPosition,((Math.ceil(Math.abs(FrontRight)) > 0.0 && Math.abs(motorFrontRight.getCurrentPosition() - frontRightTargetPosition) > tolerance)));//, frontRightTargetPosition);
//            telemetry.addData("front left", "=%.2f %d %b", FrontLeft, motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition,((Math.ceil(Math.abs(FrontLeft)) > 0.0 && Math.abs(motorFrontLeft.getCurrentPosition() - frontLeftTargetPosition) > tolerance)));//, frontLeftTargetPosition);
//            telemetry.addData("back left", "=%.2f %d %b",  BackLeft, motorBackLeft.getCurrentPosition() - backLeftTargetPosition, ((Math.ceil(Math.abs(BackLeft)) > 0.0 && Math.abs(motorBackLeft.getCurrentPosition() - backLeftTargetPosition) > tolerance)));//, backLeftTargetPosition);
//            telemetry.addData("back right", "=%.2f %d %b", BackRight, motorBackRight.getCurrentPosition() - backRightTargetPosition, ((Math.ceil(Math.abs(BackRight)) > 0.0 && Math.abs(motorBackRight.getCurrentPosition() - backRightTargetPosition) > tolerance)));
//            telemetry.update();
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
    }

    public void setWheelsToEncoderMode(){
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void moveClaw(final DcMotor motor, double position){

        Thread thr = new Thread(new Runnable() {
            @Override
            public void run() {
//                while() //while the motor is not at the position specified which would either be the top position or 0(Where it starts at the bottor)
                new Handler(Looper.getMainLooper()).post(new Runnable() {
                    @Override
                    public void run() {

                    }
                });
                }

        });
        thr.start();
        }
    ///

    //Drive Routines
    public void twoWheelDrive(double leftInput, double rightInput,int mode) {
        double rightDrive = scaleInput(rightInput,mode);
        double leftDrive = scaleInput(-leftInput,mode);

        double finalRight = Range.clip(rightDrive, -1, 1);
        double finalLeft = Range.clip(leftDrive, -1, 1);

        // write the values to the motors
        motorLeft.setPower(finalLeft);
        motorRight.setPower(finalRight);
    }

    void driveFixedDistance(double directionDrive, double inches, boolean isFast) {
        motorRight.setMode(STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setMode(STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double rotations = inches / INCHES_PER_ROTATION;
        int targetPositionR = (int) (motorRight.getCurrentPosition() + (directionDrive * rotations * COUNTS_PER_DRIVE_MOTOR_REV));
        int targetPositionL = (int) (motorLeft.getCurrentPosition() + (directionDrive * rotations * COUNTS_PER_DRIVE_MOTOR_REV));
        telemetry.addData("Drive Position R",  "= %d  %d  %d", motorRight.getCurrentPosition(),motorRight.getTargetPosition(),targetPositionR);
        telemetry.addData("Drive Position L",  "= %d  %d  %d", motorLeft.getCurrentPosition(),motorLeft.getTargetPosition(),targetPositionL);
        telemetry.update();
        while((/*touchSensor.getState() == false &&*/
                Math.abs(motorRight.getCurrentPosition() - (targetPositionR)) >INCREMENT_DRIVE_MOTOR_MOVE)
                || Math.abs(motorLeft.getCurrentPosition() - (targetPositionL)) >INCREMENT_DRIVE_MOTOR_MOVE) {
            motorRight.setTargetPosition((int) (motorRight.getCurrentPosition() + (int) directionDrive*INCREMENT_DRIVE_MOTOR_MOVE));
            if (isFast) {
                motorRight.setPower(DRIVE_MOTOR_POWER * 2);
            } else {
                motorRight.setPower(DRIVE_MOTOR_POWER);
            }
            motorLeft.setTargetPosition((int) (motorRight.getCurrentPosition() + (int) directionDrive*INCREMENT_DRIVE_MOTOR_MOVE));
            if (isFast) {
                motorLeft.setPower(DRIVE_MOTOR_POWER * 2);
            } else {
                motorLeft.setPower(DRIVE_MOTOR_POWER);
            }
            telemetry.addData("Drive Position R",  "= %d  %d  %d", motorRight.getCurrentPosition(),motorRight.getTargetPosition(),targetPositionR);
            telemetry.addData("Drive Position L",  "= %d  %d  %d", motorLeft.getCurrentPosition(),motorLeft.getTargetPosition(),targetPositionL);
            telemetry.update();

        }
        motorRight.setPower(0.0);
        motorRight.setMode(RUN_WITHOUT_ENCODER);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft.setPower(0.0);
        motorLeft.setMode(RUN_WITHOUT_ENCODER);
    }

    void driveAngledDistance(double directionDrive, double inches, boolean isFast) {
        double WEIGHT_R=1.2;
        motorRight.setMode(STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setMode(STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double rotations = inches / INCHES_PER_ROTATION;
        int targetPositionR = (int) (motorRight.getCurrentPosition() + (directionDrive * WEIGHT_R*rotations * COUNTS_PER_DRIVE_MOTOR_REV));
        int targetPositionL = (int) (motorLeft.getCurrentPosition() + (directionDrive * rotations * COUNTS_PER_DRIVE_MOTOR_REV));
        telemetry.addData("Drive Position R",  "= %d  %d  %d", motorRight.getCurrentPosition(),motorRight.getTargetPosition(),targetPositionR);
        telemetry.addData("Drive Position L",  "= %d  %d  %d", motorLeft.getCurrentPosition(),motorLeft.getTargetPosition(),targetPositionL);
        telemetry.update();
        while((/*touchSensor.getState() == false &&*/
                Math.abs(motorRight.getCurrentPosition() - (targetPositionR)) >INCREMENT_DRIVE_MOTOR_MOVE)
                || Math.abs(motorLeft.getCurrentPosition() - (targetPositionL)) >INCREMENT_DRIVE_MOTOR_MOVE) {
            motorRight.setTargetPosition((int) (motorRight.getCurrentPosition() + (int) directionDrive*INCREMENT_DRIVE_MOTOR_MOVE));
            if (isFast) {
                motorRight.setPower(WEIGHT_R*DRIVE_MOTOR_POWER * 2);
            } else {
                motorRight.setPower(WEIGHT_R*DRIVE_MOTOR_POWER);
            }
            motorLeft.setTargetPosition((int) (motorRight.getCurrentPosition() + (int) directionDrive*INCREMENT_DRIVE_MOTOR_MOVE));
            if (isFast) {
                motorLeft.setPower(DRIVE_MOTOR_POWER * 2);
            } else {
                motorLeft.setPower(DRIVE_MOTOR_POWER);
            }
            telemetry.addData("Drive Position R",  "= %d  %d  %d", motorRight.getCurrentPosition(),motorRight.getTargetPosition(),targetPositionR);
            telemetry.addData("Drive Position L",  "= %d  %d  %d", motorLeft.getCurrentPosition(),motorLeft.getTargetPosition(),targetPositionL);
            telemetry.update();

        }
        motorRight.setPower(0.0);
        motorRight.setMode(RUN_WITHOUT_ENCODER);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft.setPower(0.0);
        motorLeft.setMode(RUN_WITHOUT_ENCODER);
    }



    void driveFixedDegrees(double directionDrive, double degrees) {
        motorRight.setMode(STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double rotations = degrees / DEG_PER_ROTATION;
        int targetPositionR = (int) (motorRight.getCurrentPosition() + (directionDrive * rotations *
                COUNTS_PER_DRIVE_MOTOR_REV));
        int targetPositionL = (int) (motorLeft.getCurrentPosition() + (directionDrive * rotations *
                COUNTS_PER_DRIVE_MOTOR_REV));
        telemetry.addData("Drive Position R",  "= %d  %d  %d", motorRight.getCurrentPosition(),motorRight.getTargetPosition(),targetPositionR);
        telemetry.addData("Drive Position L",  "= %d  %d  %d", motorLeft.getCurrentPosition(),motorLeft.getTargetPosition(),targetPositionL);
        telemetry.update();
        while((/*touchSensor.getState() == false &&*/
                Math.abs(motorRight.getCurrentPosition() - (targetPositionR)) >INCREMENT_DRIVE_MOTOR_MOVE)
                || Math.abs(motorLeft.getCurrentPosition() - (targetPositionL)) >INCREMENT_DRIVE_MOTOR_MOVE) {
            motorRight.setTargetPosition((int) (motorRight.getCurrentPosition() + (int) directionDrive*INCREMENT_DRIVE_MOTOR_MOVE));
            motorRight.setPower(DRIVE_MOTOR_POWER);
            motorLeft.setTargetPosition((int) (motorRight.getCurrentPosition() + (int) directionDrive*INCREMENT_DRIVE_MOTOR_MOVE));
            motorLeft.setPower(DRIVE_MOTOR_POWER);
            telemetry.addData("Drive Position R",  "= %d  %d  %d", motorRight.getCurrentPosition(),motorRight.getTargetPosition(),targetPositionR);
            telemetry.addData("Drive Position L",  "= %d  %d  %d", motorLeft.getCurrentPosition(),motorLeft.getTargetPosition(),targetPositionL);
            telemetry.update();

        }
        motorRight.setPower(0.0);
        motorRight.setMode(RUN_WITHOUT_ENCODER);
        motorLeft.setPower(0.0);
        motorLeft.setMode(RUN_WITHOUT_ENCODER);
    }
    //Lift Routines
    /*TODO:Here use the touch sensor to detect that the lift has moved to zero
    TODO  While not pressed, move lift ok
            */
    //            while(touchSensor.getState() == false){
      public void liftDrive(int direction) {
            //  if (liftSensor.getState())
            if (direction == 1) {
                liftMotor.setPower(LIFT_MOTOR_POWER);
            } else if (direction == -1) {
                liftMotor.setPower(-LIFT_MOTOR_POWER);
            } else {
                liftMotor.setPower(0);
            }
        }



    void runliftFixedDistance(double directionLift, double rotations) {
        liftMotor.setMode(STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int targetPosition = (int) (liftMotor.getCurrentPosition() + (rotations * COUNTS_PER_MOTOR_REV));
        while((/*touchSensor.getState() == false &&*/ Math.abs(liftMotor.getCurrentPosition() - (directionLift * targetPosition)) >INCREMENT_MOTOR_MOVE)) {
            liftMotor.setTargetPosition((int) (liftMotor.getCurrentPosition() + (int) directionLift*INCREMENT_MOTOR_MOVE));
            liftMotor.setPower(directionLift * LIFT_MOTOR_POWER);
            telemetry.addData("Lift Position",  "= %d  %d  %d", liftMotor.getCurrentPosition(),liftMotor.getTargetPosition(),targetPosition);
            telemetry.update();

        }
        liftMotor.setPower(0.0);
        liftMotor.setMode(RUN_WITHOUT_ENCODER);
    }

    //Retrieval Routines
    public void servoSet(Servo myServo,double pos){
        double newPos = Range.clip(pos, -0.5, 0.5);
        myServo.setPosition(newPos);
        telemetry.addData(myServo.getDeviceName(),  "= %.2f", newPos);
        telemetry.update();
    }

    public void armDrive(int direction) {
            if(direction == 1) {
                if (touchSensor.getState()) {
                    armMotor.setPower(ARM_MOTOR_POWER);
                }
            } else if (direction == -1) {
                armMotor.setPower(-ARM_MOTOR_POWER);
            } else {
                armMotor.setPower(0);
            }
    }

    void runArmFixedDistance(double directionArm, double rotations) {
        armMotor.setMode(STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        int targetPosition = (int) (armMotor.getCurrentPosition() + (directionArm*rotations * COUNTS_PER_MOTOR_REV));
        while((Math.abs(armMotor.getCurrentPosition() - (/*directionArm * */targetPosition)) > INCREMENT_MOTOR_MOVE)) {
            armMotor.setTargetPosition((int) (armMotor.getCurrentPosition() + (int) directionArm*INCREMENT_MOTOR_MOVE));
            if ((touchSensor.getState() && directionArm == 1) || (directionArm == -1)) {
                armMotor.setPower(/*directionArm * */ARM_MOTOR_POWER);
            }
            telemetry.addData("Arm Position",  "= %d  %d  %d", armMotor.getCurrentPosition(),armMotor.getTargetPosition(),targetPosition);
            telemetry.update();

        }
        armMotor.setPower(0.0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    //Sensors
    public void printColorDistance (ColorSensor sensorColor, DistanceSensor sensorDistance){
        final double SCALE_FACTOR = 255;
        float hsvValues[] = {0F, 0F, 0F};

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
      //  telemetry.addData("Alpha", sensorColor.alpha());
      //  telemetry.addData("Red  ", sensorColor.red());
      //  telemetry.addData("Green", sensorColor.green());
      //  telemetry.addData("Blue ", sensorColor.blue());
      //  telemetry.addData("Sat", hsvValues[1]);

        telemetry.update();
    }

    //Utils
    static double scaleInput(double dVal, int mode) {
        double[] scaleArray= {};

        if(mode == 0) {
            scaleArray = new double[] {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                    0.30, 0.36, 0.4883, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};
        } else if(mode  == 1) {
            scaleArray = new double[]   {0.0, 0.002, 0.002, 0.006, 0.015, 0.03, 0.05, 0.08,
                    0.125, 0.17, 0.24, 0.3, 0.4, 0.5, 0.67, 0.82, 1.00};
        } else if(mode == 2) {
            scaleArray = new double[]  {0.0, 0.0, 0.003, 0.01, 0.03, 0.06, 0.1, 0.167,
                    0.25, 0.36, 0.43, 0.6499, 0.84, 1.00, 1.00, 1.00, 1.00};
        } else if(mode == 3) {
            scaleArray = new double[]{0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375,
                    0.4375, 0.5, 0.5625, 0.625, 0.6875, 0.75, 0.8125, 0.875, 0.9375, 1};
        } else if(mode == 4) {
            //this mode scales down the speed except for highest power since Rev motor is higher torque
            scaleArray = new double[]  {0.0, 0.002, 0.003, 0.01, 0.03, 0.06, 0.1, 0.167,
                    0.25, 0.36, 0.43, 0.5, 0.6, 0.65, 0.7, 0.75, 1.00};
        }
        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        // index should be positive.
        if (index < 0) {
            index = -index;
        }
        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }
        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        // return scaled value.
        return dScale;
    }
}
