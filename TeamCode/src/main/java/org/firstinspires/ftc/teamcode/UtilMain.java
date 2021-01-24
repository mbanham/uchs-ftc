package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.os.Handler;
import android.os.Looper;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

public class UtilMain {
    public static final String STONE = "STONE";
    public static final String SKYSTONE = "SKYSTONE";
    static final double COUNTS_PER_MOTOR_REV = 1250.0; // HD Hex Motor (REV-41-1301) 40:1
    static final double COUNTS_PER_DRIVE_MOTOR_REV = 180; // counts per reevaluation of the motor
    static final double INCREMENT_MOTOR_MOVE = 175.0; // move set amount at a time
    static final int COUNTS_PER_INCH = (int) ((1.4142 * (COUNTS_PER_DRIVE_MOTOR_REV)) / (4.0 * Math.PI)); // for 45deg
    // vuforia
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String VUFORIA_KEY = "ASVozkX/////AAAAGX+Aimqfn0YRqafZGVaD2MIhBsmxiHLTd4r2XyoV4F/VEvRMnL1mLn7NDtl1onYGhHmJADQR8nt0aX4rZLIAb/7+XxI7LLZV4X0tMBDQyBL6IWcEdgMD63hTKncdP8NsIVJxJOY971/5pVdU50XisgiiAhq3b6D9twKLfGZ9EI2M4XXM0B7BxdA7x7YMD5QcMDf96myKGsPhVlkwz8XvBdbnOvZZg2FoxmhqExRp33AKii1GZRDwvfeco0hEOKusdwOkjbJ5RTJ+9T3fAysvqSovSG8iAWZ98qrG2xop2gK73UPJaY4vj5/1yVBKFMnWt42P931ybmEW1/c5dc8LR1CyD8jCxlgqypf9oCz/q89j";
    private final double LIFT_MOTOR_POWER = 0.65;
    private final double ARM_MOTOR_POWER = 0.15;
    float hsvValues[] = {0F, 0F, 0F};
    float values[];
    private DcMotor liftMotor;
    private DcMotor armMotor;
    private Telemetry telemetry;
    private DigitalChannel touchSensor;
    private DigitalChannel liftSensor;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    // wheels
    // initialize these in InitExtraSensors if using
    private DistanceSensor colorSensor;
    private int relativeLayoutId;

    ///

    //#region Initialization
    public UtilMain(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    // Utils
    static double scaleInput(double dVal, int mode) {
        double[] scaleArray = {};

        if (mode == 0) {
            scaleArray = new double[]{0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.4883, 0.50, 0.60,
                    0.72, 0.85, 1.00, 1.00};
        } else if (mode == 1) {
            scaleArray = new double[]{0.0, 0.002, 0.002, 0.006, 0.015, 0.03, 0.05, 0.08, 0.125, 0.17, 0.24, 0.3, 0.4,
                    0.5, 0.67, 0.82, 1.00};
        } else if (mode == 2) {
            scaleArray = new double[]{0.0, 0.0, 0.003, 0.01, 0.03, 0.06, 0.1, 0.167, 0.25, 0.36, 0.43, 0.6499, 0.84,
                    1.00, 1.00, 1.00, 1.00};
        } else if (mode == 3) {
            scaleArray = new double[]{0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375, 0.4375, 0.5, 0.5625, 0.625,
                    0.6875, 0.75, 0.8125, 0.875, 0.9375, 1};
        } else if (mode == 4) {
            // this mode scales down the speed except for highest power since Rev motor is
            // higher torque
            scaleArray = new double[]{0.0, 0.002, 0.003, 0.01, 0.03, 0.06, 0.1, 0.167, 0.25, 0.36, 0.43, 0.5, 0.6,
                    0.65, 0.7, 0.75, 1.00};
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
    //#endregion

    static double MathLimit(double min, double max, double value) {
        if (value >= min && value <= max) {
            return value;
        } else if (value < min) {
            return min;
        } else if (value > max) {
            return max;
        }
        return -99999;
    }

    static double[] LimitList(double min, double max, double[] list) {
        //declaring variables
        double list_max = 0;// Ex. 8
        double list_min = 0;// Ex. -2
        double range = max - min;//range of output  Ex. 10

        for (int i = 0; i < list.length; i++) {//get maximum and minimum of lisst
            double d = list[i];
            if (d > list_max)
                list_max = d;
            if (d < list_min)
                list_min = d;
        }
        list_max -= list_min;
        double[] new_list = new double[]{};
        for (int i = 0; i < list.length; i++) {
            double d = list[i];//get value Ex. 3 in range -2 8
            d -= list_min; // Ex. 3-(-2) = 5 of 10
            double pt = d / list_max;//get Ex. 5/10
            new_list[i] = (range * pt) + min; // Ex. min=2 range= 20  20*5/10 = 10 + 2 = 12

        }
        return new_list;
    }

    //not being used in 2020 config
    public void InitExtraSensors(HardwareMap hardwareMap) {
        // get a reference to the color sensor.
        colorSensor = hardwareMap.get(DistanceSensor.class, "distance");
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

    //#region Vuforia PatternRecog
    public void InitVuforia(HardwareMap hwm) {
        initVuforiaRaw();

        // if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
        //      initTfod(hwm);
        //   } else {
        //       telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        //   }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command. Do
         * it here so that the Camera Stream window will have the TensorFlow annotations
         * visible.
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
         * Configure Vuforia by creating a Parameter object, and passing it to the
         * Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection
        // engine.
    }

    private void initTfod(HardwareMap hwm) {
        int tfodMonitorViewId = hwm.appContext.getResources().getIdentifier("tfodMonitorViewId", "id",
                hwm.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE, SKYSTONE);
    }

    public StoneElement[] GetObjectsInFrame() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        StoneElement[] elem = new StoneElement[]{};
        for (int i = 0; i < updatedRecognitions.size(); i++) {
            elem[i] = new StoneElement(updatedRecognitions.get(i));

        }
        return elem;
    }

    //#endregion
    public void moveClaw(final DcMotor motor, double position) {

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

    // Sensors
    public void printColorDistance(ColorSensor sensorColor, DistanceSensor sensorDistance) {
        final double SCALE_FACTOR = 255;
        float hsvValues[] = {0F, 0F, 0F};

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR), (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR), hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        // telemetry.addData("Alpha", sensorColor.alpha());
        // telemetry.addData("Red ", sensorColor.red());
        // telemetry.addData("Green", sensorColor.green());
        // telemetry.addData("Blue ", sensorColor.blue());
        // telemetry.addData("Sat", hsvValues[1]);

        telemetry.update();
    }

    //#endregion
    public void liftDrive(int direction) {
        // if (liftSensor.getState())
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
        while ((/* touchSensor.getState() == false && */ Math
                .abs(liftMotor.getCurrentPosition() - (directionLift * targetPosition)) > INCREMENT_MOTOR_MOVE)) {
            liftMotor.setTargetPosition(
                    (int) (liftMotor.getCurrentPosition() + (int) directionLift * INCREMENT_MOTOR_MOVE));
            liftMotor.setPower(directionLift * LIFT_MOTOR_POWER);
            telemetry.addData("Lift Position", "= %d  %d  %d", liftMotor.getCurrentPosition(),
                    liftMotor.getTargetPosition(), targetPosition);
            telemetry.update();

        }
        liftMotor.setPower(0.0);
        liftMotor.setMode(RUN_WITHOUT_ENCODER);
    }

    // Retrieval Routines
    public void servoSet(Servo myServo, double pos) {
        double newPos = Range.clip(pos, -0.5, 0.5);
        myServo.setPosition(newPos);
        telemetry.addData(myServo.getDeviceName(), "= %.2f", newPos);
        telemetry.update();
    }

    public void armDrive(int direction) {
        if (direction == 1) {
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
        int targetPosition = (int) (armMotor.getCurrentPosition() + (directionArm * rotations * COUNTS_PER_MOTOR_REV));
        while ((Math
                .abs(armMotor.getCurrentPosition() - (/* directionArm * */targetPosition)) > INCREMENT_MOTOR_MOVE)) {
            armMotor.setTargetPosition(
                    (int) (armMotor.getCurrentPosition() + (int) directionArm * INCREMENT_MOTOR_MOVE));
            if ((touchSensor.getState() && directionArm == 1) || (directionArm == -1)) {
                armMotor.setPower(/* directionArm * */ARM_MOTOR_POWER);
            }
            telemetry.addData("Arm Position", "= %d  %d  %d", armMotor.getCurrentPosition(),
                    armMotor.getTargetPosition(), targetPosition);
            telemetry.update();

        }
        armMotor.setPower(0.0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    ////#endregion
}
