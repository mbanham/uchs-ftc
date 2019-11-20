package org.firstinspires.ftc.team11288;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;


@Autonomous(name = "LinearExample", group = "Linear Opmode")
//@Disabled                            // Comment this out to add to the opmode list

public class LinearExample extends LinearOpMode {
    //initialize these variables, override them in the constructor
    private int TEAM_COLOR = Color.BLUE;
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
    private static final double START_ARM_SERVO_L = 0.8;
    private static final double START_ARM_SERVO_R = 0.1;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private static final double OPEN_CLAW_SERVO_L = 0.7;
    private static final double OPEN_CLAW_SERVO_R = 0.3;
    private static final double START_CLAW_SERVO_L = 0.86;
    private static final double START_CLAW_SERVO_R = 0.14;
    private double clawOffsetL = START_CLAW_SERVO_L;                  // Init to closed position
    private double clawOffsetR = START_CLAW_SERVO_R;                  // Init to closed position

    private double armOffsetL = START_ARM_SERVO_L;                  // Init to down position
    private double armOffsetR = START_ARM_SERVO_R;                  // Init to down position
    private final double CLAW_SPEED = 0.01;                 // sets rate to move servo
    private final double ARM_SPEED = 0.01;                 // sets rate to move servo


    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;
    private DigitalChannel touchSensor;



 //    DigitalChannel liftSensor = hardwareMap.get(DigitalChannel.class, "lift limit");


    private ElapsedTime runtime = new ElapsedTime();

    int detectedColor = Color.WHITE;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        //servos for lift
        leftArm = hardwareMap.servo.get("left arm");
        rightArm = hardwareMap.servo.get("right arm");
        leftArm.setPosition(START_ARM_SERVO_L);
        rightArm.setPosition(START_ARM_SERVO_R);

        //motor for arm
        armMotor = hardwareMap.dcMotor.get("arm motor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(0);
        //initialize claw servos
        leftClaw = hardwareMap.servo.get("left claw");
        rightClaw = hardwareMap.servo.get("right claw");
        leftClaw.setPosition(START_CLAW_SERVO_L);
        rightClaw.setPosition(START_CLAW_SERVO_R);

        //color sensor- needed?
        //  sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        //  sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        //TODO touch sensor
        touchSensor = hardwareMap.get(DigitalChannel.class, "arm limit");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        //utils class initializer
   //     teamUtils = new Util(motorRight, motorLeft, liftMotor, armMotor, telemetry, touchSensor);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Play started
        runtime.reset();
        boolean doneWithCamera = false;
        boolean doneKnockingGold = false;
        boolean doneWithDrive = false;

        //land system
//        teamUtils.runliftFixedDistance(1.0, 2.0);
//        try {
//            Thread.sleep(1000);
//        } catch (InterruptedException e) {
//
//        }
        teamUtils.runliftFixedDistance(1.0, 4.5);
        //back up

       // teamUtils.runArmFixedDistance(1.0, 0.25);
     //   teamUtils.armDrive(1);
        teamUtils.driveAngledDistance(-1.0, 7.0, false);
        teamUtils.runliftFixedDistance(-1.0, 4.5);
        teamUtils.driveFixedDegrees(1, 40);
        teamUtils.driveFixedDistance(1.0, 7.0, false);
        //ask camera where gold is
        //read where gold marker located
       // FindGoldPosition findMineral = new FindGoldPosition(telemetry, hardwareMap);
        int locationOfGold = 2; //unknown

        int error = 0;
        boolean madeError = false;
        final int turn_degrees = 13;
        final int iterations = 12;

        while (opModeIsActive()) {
            // run this loop until the end of the match (driver presses stop)
            //find+knock off gold mineral
            if (runtime.milliseconds() < 20000 && locationOfGold > 1) {
              //  locationOfGold = findMineral.getGoldPosition();
                if(locationOfGold == 3) {
                    //sees 2 minerals
                    if(madeError) {
                        teamUtils.driveFixedDegrees(1, turn_degrees);
                    } else {
                        teamUtils.driveFixedDegrees(-1, turn_degrees);
                        error += turn_degrees;
                    }
                    //take multiple readings after adjusting
                    for (int i = 0; i < iterations; i++) {
                      //  locationOfGold = findMineral.getGoldPosition();
                        try {
                            Thread.sleep(100);
                        } catch (Exception E){
                        }
                    }
                    //continue around in loop again
                }
                else if(locationOfGold == 4) {
                    //sees 1 mineral
                    if(!madeError) {
                        teamUtils.driveFixedDegrees(1, error);
                    }
                    //take multiple readings after adjusting
                    for (int i = 0; i < iterations; i++) {
                 //       locationOfGold = findMineral.getGoldPosition();
                        try {
                            Thread.sleep(100);
                        } catch (Exception E){

                        }
                    }
                    madeError = true;
                    //continue around in loop again
                }
            } else if(!doneWithCamera){
               // findMineral.close(); //done with camera;
                doneWithCamera=true;
            }
            telemetry.addData("Gold Position:", locationOfGold);

                //now check where is the mineral
                //locationOfGold=1 - LEFT
                //locationOfGold=0 - CENTER
                //locationOfGold=-1 - RIGHT

            if(doneWithCamera && !doneKnockingGold) {
                //depending on locationoOfGold
                if(locationOfGold == 0) {  //center
                    teamUtils.driveFixedDegrees(1,90);
                    teamUtils.driveFixedDistance(1, 32, false);
                    teamUtils.driveFixedDegrees(1, 0);
                    teamUtils.driveFixedDistance(1,12, false);
                    teamUtils.driveFixedDegrees (1, 10);
                    doneKnockingGold=true;
                    //drop Marker

                } else if (locationOfGold == 1) {  //left
                    teamUtils.driveFixedDegrees(1,120);
                    teamUtils.driveFixedDistance(1, 32, false);
                    teamUtils.driveFixedDegrees(-1,50);
                    teamUtils.driveFixedDistance(1,13.4, false);
                    doneKnockingGold=true;
                    //drop Marker

                } else if (locationOfGold == -1) {  //right
                    teamUtils.driveFixedDegrees (1, 60);
                    teamUtils.driveFixedDistance(1, 32, false);
                    teamUtils.driveFixedDegrees(1,40);
                    teamUtils.driveFixedDistance(1,13.4, false);
                    doneKnockingGold=true;
                    //drop Marker

                } else { //not detected
                    telemetry.addData("Status", "rarted");
                    teamUtils.driveFixedDegrees(1,90);
                    teamUtils.driveFixedDistance(1, 32, false);
                    teamUtils.driveFixedDegrees(1, 0);
                    teamUtils.driveFixedDistance(1,12, false);
                    teamUtils.driveFixedDegrees (1, 10);
                    doneKnockingGold=true;
                    //telemetry.addData("Status","Peter's code has failed us");
                //    teamUtils.driveFixedDegrees(1, 150);
               //     teamUtils.driveFixedDistance(1,24);
                //    teamUtils.driveFixedDegrees(1,-90);
               //     teamUtils.driveFixedDistance(1,24);
                }

            }


            if(doneWithCamera && doneKnockingGold && !doneWithDrive) {
                teamUtils.driveFixedDistance(1,6, true);
                doneWithDrive=true;
            }


        //    telemetry.update();
            }
        //   //be sure to shutdown camera;
        //   findMineral.close();
        }

    }
