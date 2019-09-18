// Team11288_Teleop
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This file provides Teleop driving for the Team11288 robot.
 * The code is structured as an Iterative OpMode
 *
 *
 */

@TeleOp(name="Team11288_Test_Gold_Position", group="Teleop")
@Disabled
public class Team11288_Test_Gold_Position extends OpMode{

    private static final int teleopType1 = 0, teleopType2 = 1, teleopType3 = 2, teleopTypeLinear = 3, teleopTypeRev = 4;
    private int currentScaleInputMode = teleopTypeLinear;

    /* Declare OpMode members. */
  //wheels
  //  private DcMotor motorFrontRight;
   // private DcMotor motorFrontLeft;
    private DcMotor motorRight;
    private DcMotor motorLeft;
    private DcMotor liftMotor;
    private DcMotor armMotor;
    private Util teamUtils;

    //claw and arm
    private DcMotor shoulder; //bottom pivot of the new claw
    private int currentPosition; //used to track shoulder motor current position
    private int targetPosition; //used to track target shoulder position
    private double minPosition; //minimum allowed position of shoulder motor
    private double maxPosition; //maximum allowed positon of shoulder motor


    private Servo leftArm = null;
    private Servo rightArm = null;
    private static final double START_ARM_SERVO_L = 1.0;
    private static final double START_ARM_SERVO_R = 0.0;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private static final double START_CLAW_SERVO_L = 0.7;
    private static final double START_CLAW_SERVO_R = 0.3;

    private double clawOffsetL = START_CLAW_SERVO_L;                  // Init to closed position
    private double clawOffsetR = START_CLAW_SERVO_R;                  // Init to closed position

    private double armOffsetL = START_ARM_SERVO_L;                  // Init to down position
    private double armOffsetR = START_ARM_SERVO_R;                  // Init to down position
    private final double CLAW_SPEED = 0.01;                 // sets rate to move servo
    private final double ARM_SPEED = 0.01;                 // sets rate to move servo





    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;
    FindGoldPosition findMineral;

     //TODO touch sensor
//    DigitalChannel touchSensor;  // Hardware Device Object

    /* Code to run ONCE when the driver hits INIT */
        @Override
    public void init() {
        // Initialize the hardware variables.
        // Send telemetry message to signify robot waiting
            telemetry.addData("Say", "Hello Driver");

            //initialize wheels
            motorLeft = hardwareMap.dcMotor.get("motor left");
            motorRight = hardwareMap.dcMotor.get("motor right");
            motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
            motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //lift dc motor for lift system
            //liftMotor or "lift motor"
            liftMotor = hardwareMap.dcMotor.get("lift motor");
            liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //servos for arm
            leftArm = hardwareMap.servo.get("left arm");
            rightArm = hardwareMap.servo.get("right arm");
            leftArm.setPosition(START_ARM_SERVO_L);
             rightArm.setPosition(START_ARM_SERVO_R);

            //initialize claw servos
            leftClaw  = hardwareMap.servo.get("left claw");
            rightClaw = hardwareMap.servo.get("right claw");
            leftClaw.setPosition(START_CLAW_SERVO_L);
            rightClaw.setPosition(START_CLAW_SERVO_R);

            armMotor = hardwareMap.dcMotor.get("arm motor");
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            //color sensor- needed?
          //  sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
          //  sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

            //utils class initializer
            //teamUtils = new Util(motorRight, motorLeft, liftMotor, armMotor, telemetry);
            findMineral = new FindGoldPosition(telemetry,hardwareMap);

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
    int locationOfGold = 2; //unknown

    @Override
    public void loop() {






                // run this loop until the end of the match (driver presses STOP)
                       //find+knock off gold mineral
                if ( locationOfGold == 2) {
                    locationOfGold = findMineral.getGoldPosition();
                } else {
                    findMineral.close(); //done with camera;

                }
                telemetry.addData("Gold Position:", locationOfGold);
                telemetry.update();



        //            //initialize touch sensor
//            touchSensor = hardwareMap.get(DigitalChannel.class, "sensor_touch");
//            // set the digital channel to input.
//            touchSensor.setMode(DigitalChannel.Mode.INPUT);

               /*
            TODO:Here use the touch sensor to detect that the arm has moved to zero
            TODO  While not pressed, move arm back, should ensure the knocking arm out of way
            */
//            while(touchSensor.getState() == false){
//                targetPosition = shoulder.getCurrentPosition() - (int) (INCREMENT_MOTOR_MOVE);
//                shoulder.setTargetPosition(targetPosition);
//                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                shoulder.setPower(SHOULDER_POWER);
//            }
        //now set the minposition here
        //    currentPosition = shoulder.getCurrentPosition(); //assume that we start with the shoulder down all the way - this is zero
        //    minPosition = currentPosition;
        //allow about 100deg of motion total
        //    maxPosition = minPosition + INCREMENT_MOTOR_MOVE*17.0;
        //    shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //   shoulder.setPower(0);



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

}
