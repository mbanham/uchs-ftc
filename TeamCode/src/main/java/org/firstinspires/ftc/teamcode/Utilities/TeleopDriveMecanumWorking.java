// Team11288_Teleop
package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This file provides Teleop driving for the Team11288 TeleopDrive drive robot.
 * The code is structured as an Iterative OpMode
 *
 * Assumes claw with arm having shoulder motor, elbow servo and wrist servo - all having 180deg servos
 *
 */
@TeleOp(name = "TeleopDriveMecanumWorking", group = "Teleop")
public class TeleopDriveMecanumWorking extends OpMode {

    //claw and arm
    //  static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // NeveRest Classic 40 Gearmotor (am-2964a)
    static final double INCREMENT_MOTOR_MOVE = 100; // move about 10 degrees at a time
    static final double COUNTS_PER_MOTOR_REV = 1250.0; //HD Hex Motor (REV-41-1301) 40:1
    static final double COUNTS_PER_DRIVE_MOTOR_REV = 300.0; // counts per reevaluation of the motor
    static final double INCREMENT_DRIVE_MOTOR_MOVE = 30.0; // move set amount at a time
    static final double MOTOR_STRAFE_INFLUENCE = 1;
    private static final double SAFE_ARM_POSITION = 0.0;
    private final double ARM_MOTOR_POWER = 0.5;
    private final double DRIVE_MOTOR_POWER = 0.75;
    //color sensorl
    NormalizedColorSensor colorSensor;
    double distanceToDrive = 6.0;  //test parameter
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double speed_multiplier = 1;
    double arm_multiplier = 1;
    /* Declare OpMode members. */
    //wheels
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
//    private DcMotor motorArm;
//    private DcMotor motorLift;
    private UtilHolonomic teamUtils;
//    private DcMotor shoulder; //bottom pivot of the new claw
    private DcMotor motorLauncher;
    private DcMotor motorLoader;
    private Servo loader = null, wobbler = null;
    private int currentPosition; //used to track shoulder motor current position
    private int targetPosition; //used to track target shoulder position
    private double minPosition; //minimum allowed position of shoulder motor
    private double maxPosition; //maximum allowed positon of shoulder motor

    //TODO touch sensor
    //DigitalChannel touchSensor;  // Hardware Device Object
//    private elbow             = null;
//    private Servo wrist       = null;
//    private Servo claw = null;
//    private Servo platform = null;
    //arm for knocking jewel - keep it out of the way in Driver Mode
    private Servo knockingArm = null;
    private int directionArm = 1;
    private int rotations = 12;
    private int initialPosition;

    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void init() {
        // Initialize the hardware variables.
        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Hello Driver");

        //initialize wheels
        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");
//        motorArm = hardwareMap.dcMotor.get("motor arm");
//        claw = hardwareMap.servo.get("claw servo");
//        platform = hardwareMap.servo.get("platform servo");
//        motorLift = hardwareMap.dcMotor.get("motor lift");
        motorLauncher = hardwareMap.dcMotor.get("motor launcher");
        motorLoader = hardwareMap.dcMotor.get("motor loader");
        loader = hardwareMap.servo.get("launch servo");
        wobbler = hardwareMap.servo.get("wobble servo");

        loader.setPosition(0.3);
        wobbler.setPosition(0.7);

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLoader.setDirection(DcMotorSimple.Direction.FORWARD);

//        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorArm.setMode(STOP_AND_RESET_ENCODER);
//        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        motorLift.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorLift.setMode(STOP_AND_RESET_ENCODER);
//        motorLift.setDirection(DcMotorSimple.Direction.FORWARD);

       // initialPosition = (int) (motorArm.getCurrentPosition());
        rotations = 12;
        directionArm = -1;
        //targetPosition = (int) (motorArm.getCurrentPosition() + (directionArm * rotations * COUNTS_PER_MOTOR_REV));

        //utils class initializer
        teamUtils = new UtilHolonomic(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    float gear, trigger_sensitivity = 0.1f, multiplier = 1;

    double lastLoadTime;

    @Override
    public void loop() {
        gear = 1;
        if(gamepad1.left_trigger > trigger_sensitivity && gamepad1.right_trigger > trigger_sensitivity) gear = 0.5f;

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;

        double rightX = scaleInput(gamepad1.right_stick_x * multiplier);
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = -r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = -r * Math.cos(robotAngle) - rightX;

        double FrontRight = Range.clip(v2 * multiplier * gear, -1, 1);
        double FrontLeft = Range.clip(v1 * multiplier * gear, -1, 1);
        double BackLeft = Range.clip(v3 * multiplier * gear, -1, 1);
        double BackRight = Range.clip(v4 * multiplier * gear, -1, 1);

        // write the values to the motors
        teamUtils.setWheelsToSpeedMode();
        motorFrontRight.setPower(FrontRight);
        motorFrontLeft.setPower(FrontLeft);
        motorBackLeft.setPower(BackLeft);
        motorBackRight.setPower(BackRight);

        //platformArm


        if (Range.clip(gamepad1.right_trigger, 0.4, 1) > 0.4) {//from 0.4 to 1 = 0.6
            speed_multiplier = 1 - Range.clip(gamepad1.right_trigger, 0, 0.4); //speed speed_multiplier 1 - 0.6
        } else {
            speed_multiplier = 1;
        }

        if(gamepad2.a) {
            loader.setPosition(0.43);
            lastLoadTime = getRuntime();
        } else if(getRuntime() > lastLoadTime + 0.15) {
            loader.setPosition(0.35);
        }

        if(gamepad2.x) {
            wobbler.setPosition(0.7);
        } else if(gamepad2.y) {
            wobbler.setPosition(0.3);
        }

        if(gamepad2.right_trigger > 0.1) {
            motorLauncher.setPower(gamepad2.right_trigger);
        } else {
            motorLauncher.setPower(0);
        }

        if(gamepad2.left_trigger > 0.1) {
            motorLoader.setPower(gamepad2.left_trigger);
        } else {
            motorLoader.setPower(0);
        }

        //press x
        //while x is pressed, press dpad buttons to proper degree
        //release x to start with parameters
        //press x to stop the robot and debug

        /*if(gamepad1.x){
            if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                teamUtils.resetEncoderOnMotors();
                double x_int = 0.5;
                double y_int = 0.5;
                while(gamepad1.x) {
                    //y int
                    if (gamepad1.dpad_up) {
                        y_int += 0.1;
                        while(gamepad1.dpad_up) {
                        }
                        telemetry.addData("MyActivity", "val=" + y_int);
                        telemetry.update();
                    }
                    else
                    if (gamepad1.dpad_down){
                        y_int -= 0.1;
                        while(gamepad1.dpad_down) {
                        }
                        telemetry.addData("MyActivity", "val=" + y_int);
                        telemetry.update();

                    }
                    //x int
                    if (gamepad1.dpad_left) {
                        x_int -= 0.1;
                        while(gamepad1.dpad_left) {
                        }
                        telemetry.addData("MyActivity", "val=" + x_int);
                        telemetry.update();
                    }
                    else
                    if (gamepad1.dpad_right)
                    {
                        x_int += 0.1;
                        while(gamepad1.dpad_right) {
                        }
                        telemetry.addData("MyActivity", "val=" + x_int);
                        telemetry.update();
                    }
                }

                teamUtils.drivebySpeed(x_int, y_int, 0);
                while(!gamepad1.x){

                }
                teamUtils.stopWheelsSpeedMode();
                double dist_y = UtilHolonomic.COUNTS_PER_INCH * (motorBackLeft.getTargetPosition() + motorFrontLeft.getTargetPosition() + motorBackRight.getTargetPosition() + motorFrontRight.getTargetPosition()) / 4;
                double dist_x = UtilHolonomic.COUNTS_PER_INCH * (-motorBackLeft.getTargetPosition() + motorFrontLeft.getTargetPosition() + motorBackRight.getTargetPosition() + -motorFrontRight.getTargetPosition()) / 4;

                String output = x_int + "-" + y_int + Math.sqrt(Math.pow(dist_x, 2) + Math.pow(dist_y, 2));
                telemetry.addData("MyActivity", output);

            }
        }*/

        //dpad control 1 inch
//            double x_int = 0;
//            double y_int = 0;
//            if (gamepad1.dpad_up)
//                y_int = 0.3;
//            if(gamepad1.dpad_down)
//                y_int = -0.3;
//            if(gamepad1.dpad_left)
//                x_int = 0.3;
//            if(gamepad1.dpad_right)
//                x_int = -0.3;
//            if(x_int != 0 || y_int != 0) {
//                teamUtils.drivebyDistance(x_int, y_int, 1, "inch");
//            }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal) {
        //original curve
        //double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
        //      0.30, 0.36, 0.4883, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        //1/2y =1/2x^3
        //double[] scaleArray = {0.0, 0.002, 0.002, 0.006, 0.015, 0.03, 0.05, 0.08,
        // 0.125, 0.17, 0.24, 0.3, 0.4, 0.5, 0.67, 0.82, 1.00};
//1/2y = x^3
        double[] scaleArray = {0.0, 0.0, 0.003, 0.01, 0.03, 0.06, 0.1, 0.167,
                0.25, 0.36, 0.43, 0.6499, 0.84, 1.00, 1.00, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = Math.abs((int) (dVal * 16.0));
        //index cannot exceed size of array minus 1.
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

    public void drive(double lsx, double lsy, double rsx) {
        double r = Math.hypot(scaleInput(lsx), scaleInput(lsy));
        double robotAngle = Math.atan2(scaleInput(lsy), scaleInput(-lsx)) - Math.PI / 4;
        double rightX = scaleInput(rsx);
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = -r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = -r * Math.cos(robotAngle) - rightX;

        double FrontRight = Range.clip(v2, -1, 1);
        double FrontLeft = Range.clip(v1, -1, 1);
        double BackLeft = Range.clip(v3, -1, 1);
        double BackRight = Range.clip(v4, -1, 1);

        // write the values to the motors
        motorFrontRight.setPower(FrontRight);
        motorFrontLeft.setPower(FrontLeft);
        motorBackLeft.setPower(BackLeft);
        motorBackRight.setPower(BackRight);
    }
}
