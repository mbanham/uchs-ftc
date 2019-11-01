// Team11288_Teleop
package org.firstinspires.ftc.team11288;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/*
 * This file provides Teleop driving for the Team11288 Holonomic drive robot.
 * The code is structured as an Iterative OpMode
 *
 * Assumes claw with arm having shoulder motor, elbow servo and wrist servo - all having 180deg servos
 *
 */

@TeleOp(name="Holonomic Test", group="Teleop")
public class Holonomic extends OpMode{

    /* Declare OpMode members. */
  //wheels
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor motorLift;

    //claw and arm
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // NeveRest Classic 40 Gearmotor (am-2964a)
    static final double INCREMENT_MOTOR_MOVE = 30; // move about 10 degrees at a time
    private DcMotor shoulder; //bottom pivot of the new claw
    private int currentPosition; //used to track shoulder motor current position
    private int targetPosition; //used to track target shoulder position
    private double minPosition; //minimum allowed position of shoulder motor
    private double maxPosition; //maximum allowed positon of shoulder motor

//    private elbow             = null;
//    private Servo wrist       = null;
    private Servo claw        = null;
    private Servo platform    = null;
    private static final double MID_SERVO           =  0.5 ;
    private static final double INIT_ELBOW_SERVO    =  0.0 ;
    private static final double SHOULDER_POWER      =  1.0 ;
    private static final double SHOULDER_UP_POWER   =  0.5 ;
    private static final double SHOULDER_DOWN_POWER = -0.5 ;
    private double          clawOffset  = 0.4 ;                  // Init to closed position
    private final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    private double          elbowOffset  = 0.0 ;                  // Servo mid position
    private final double    ELBOW_SPEED  = 0.02 ;                  // sets rate to move servo
//    private double          wristOffset  = 0.0 ;                  // Servo mid position
//    private final double    WRIST_SPEED  = 0.02 ;                 // sets rate to move servo
    private static final double INIT_KNOCKINGARM = 0.3;   // Gets the knocking arm out of the way


    //arm for knocking jewel - keep it out of the way in Driver Mode
    private Servo knockingArm = null;
    private static final double SAFE_ARM_POSITION       =  0.0 ;
    //color sensor
    NormalizedColorSensor colorSensor;

    //TODO touch sensor
    DigitalChannel touchSensor;  // Hardware Device Object

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
            motorLift = hardwareMap.dcMotor.get("motor lift");
            claw = hardwareMap.servo.get("claw servo");
            platform = hardwareMap.servo.get("platform servo");

            claw.setPosition(0);
            platform.setPosition(0);
            motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            motorLift.setDirection(DcMotorSimple.Direction.FORWARD);

            //initialize knocking arm
            //move it out of the way and turn off the sensor light
//            knockingArm = hardwareMap.servo.get("knock arm");
//            knockingArm.setPosition(INIT_KNOCKINGARM);
//            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//            if (colorSensor instanceof SwitchableLight) {
//                ((SwitchableLight) colorSensor).enableLight(false);
//            }
//
//            //initialize touch sensor
//            touchSensor = hardwareMap.get(DigitalChannel.class, "sensor_touch");
//            // set the digital channel to input.
//            touchSensor.setMode(DigitalChannel.Mode.INPUT);
//
//            //initialize shoulder motor
//            shoulder = hardwareMap.dcMotor.get("shoulder");
//            shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
//            shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            /*
//            TODO:Here use the touch sensor to detect that the arm has moved to zero
//            TODO  While not pressed, move arm back, should ensure the knocking arm out of way
//            */
//            while(touchSensor.getState() == false){
//                targetPosition = shoulder.getCurrentPosition() - (int) (INCREMENT_MOTOR_MOVE);
//                shoulder.setTargetPosition(targetPosition);
//                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                shoulder.setPower(SHOULDER_POWER);
//            }
//            //now set the minposition here
//            currentPosition = shoulder.getCurrentPosition(); //assume that we start with the shoulder down all the way - this is zero
//            minPosition = currentPosition;
//            //allow about 100deg of motion total
//            maxPosition = minPosition + INCREMENT_MOTOR_MOVE*17.0;
//            shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            shoulder.setPower(0);
//
//            //initialize claw and arm servos
//            leftClaw  = hardwareMap.servo.get("left claw");
//            rightClaw = hardwareMap.servo.get("right claw");
//            leftClaw.setPosition(MID_SERVO+0.4);
//            rightClaw.setPosition(MID_SERVO-0.4);

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

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double r = Math.hypot(scaleInput(gamepad1.left_stick_x), scaleInput(gamepad1.left_stick_y));
        double robotAngle = Math.atan2(scaleInput(gamepad1.left_stick_y), scaleInput(-gamepad1.left_stick_x)) - Math.PI / 4;
        double rightX = scaleInput(gamepad1.right_stick_x);
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

        if(gamepad2.x) {
            platform.setPosition(0);
            Log.i("MyActivity", "ServoPosition=0");
        } else if(gamepad2.y) {
            platform.setPosition(1);
            Log.i("MyActivity", "ServoPosition=1");
        }



        motorLift.setPower(gamepad2.left_stick_y);

        // Use gamepad buttons to move the shoulder motor up (Y) and down (A)
//        if (gamepad2.y) {
//            telemetry.addData("Motor Position", targetPosition);
//            targetPosition = shoulder.getCurrentPosition() + (int) (INCREMENT_MOTOR_MOVE);
//          //  if (/*targetPosition >= minPosition && */targetPosition <= maxPosition) {
//                shoulder.setTargetPosition(targetPosition);
//                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                shoulder.setPower(SHOULDER_POWER);
//          //
//        }
//        else {
//            if (gamepad2.a) {
//                targetPosition = shoulder.getCurrentPosition() - (int) (INCREMENT_MOTOR_MOVE);
//                //if (targetPosition >= minPosition && targetPosition <= maxPosition) {
//                    shoulder.setTargetPosition(targetPosition);
//                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    shoulder.setPower(SHOULDER_POWER);
//              //  }
//            }
//                //else {
//               // if (shoulder.getCurrentPosition() > maxPosition) {
//               //    shoulder.setTargetPosition((int) maxPosition);
//               //    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//               //    shoulder.setPower(SHOULDER_POWER);
//              //  }
//                else {
//                   // shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    shoulder.setPower(0.0);
//                }
//          //  }
//        }
//
//        telemetry.addData("ARM",  "= %d", shoulder.getCurrentPosition());
//
//        // Use gamepad2 dpad up and down to move elbow up and down
//        if (gamepad2.dpad_up)
//            elbowOffset += ELBOW_SPEED;
//        else if (gamepad2.dpad_down)
//            elbowOffset -= ELBOW_SPEED;
//        elbowOffset = Range.clip(elbowOffset, -0.5, 0.5);
//        knockingArm.setPosition(MID_SERVO + elbowOffset);
//       // telemetry.addData("knockingArm",  "= %.2f", MID_SERVO + elbowOffset);
//
//        // Use gamepad2 left & right Bumpers to open and close the claw
//        if (gamepad2.right_bumper)
//            clawOffset += CLAW_SPEED;
//        else if (gamepad2.left_bumper)
//            clawOffset -= CLAW_SPEED;
//        // Move both servos to new position.  Assume servos are mirror image of each other.
//        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//        leftClaw.setPosition(MID_SERVO + clawOffset);
//        rightClaw.setPosition(MID_SERVO - clawOffset);
//        telemetry.addData("clawOffset",  "= %.2f", clawOffset);
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
