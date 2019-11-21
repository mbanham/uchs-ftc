package org.firstinspires.ftc.team11288;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;


@Autonomous(name = "LinearExample", group = "Linear Opmode")
//@Disabled                            // Comment this out to add to the opmode list

public class LinearExample extends LinearOpMode {
    //initialize these variables, override them in the constructor
    private int TEAM_COLOR = Color.BLUE;
    private static final int teleopType1 = 0, teleopType2 = 1, teleopType3 = 2, teleopTypeLinear = 3, teleopTypeRev = 4;
    private int currentScaleInputMode = teleopTypeLinear;

    /* Declare OpMode members. */
    //wheels
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor motorLift;

    private Util teamUtils;


    //claw and arm
    //  static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // NeveRest Classic 40 Gearmotor (am-2964a)
    static final double INCREMENT_MOTOR_MOVE = 100; // move about 10 degrees at a time

    private final double ARM_MOTOR_POWER = 0.5;
    private final double DRIVE_MOTOR_POWER = 0.75;
    static final double COUNTS_PER_MOTOR_REV = 1250.0; //HD Hex Motor (REV-41-1301) 40:1
    static final double COUNTS_PER_DRIVE_MOTOR_REV = 300.0; // counts per reevaluation of the motor
    static final double INCREMENT_DRIVE_MOTOR_MOVE = 30.0; // move set amount at a time

    private DcMotor shoulder; //bottom pivot of the new claw
    private int currentPosition; //used to track shoulder motor current position
    private int targetPosition; //used to track target shoulder position
    private double minPosition; //minimum allowed position of shoulder motor
    private double maxPosition; //maximum allowed positon of shoulder motor

    //    private elbow             = null;
//    private Servo wrist       = null;
    private Servo claw = null;
    private Servo platform = null;
    private static final double MID_SERVO = 0.5;
    private static final double INIT_ELBOW_SERVO = 0.0;
    private static final double SHOULDER_POWER = 1.0;
    private static final double SHOULDER_UP_POWER = 0.5;
    private static final double SHOULDER_DOWN_POWER = -0.5;
    private double clawOffset = 0.4;                  // Init to closed position
    private final double CLAW_SPEED = 0.02;                 // sets rate to move servo
    private double elbowOffset = 0.0;                  // Servo mid position
    private final double ELBOW_SPEED = 0.02;                  // sets rate to move servo
    //    private double          wristOffset  = 0.0 ;                  // Servo mid position
//    private final double    WRIST_SPEED  = 0.02 ;                 // sets rate to move servo
    private static final double INIT_KNOCKINGARM = 0.3;   // Gets the knocking arm out of the way


    //arm for knocking jewel - keep it out of the way in Driver Mode
    private Servo knockingArm = null;
    private static final double SAFE_ARM_POSITION = 0.0;
    //color sensorl
    NormalizedColorSensor colorSensor;

    //TODO touch sensor
    DigitalChannel touchSensor;  // Hardware Device Object


    private int directionArm = 1;
    private int rotations = 12;
    private int initialPosition;


    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(STOP_AND_RESET_ENCODER);
        motorLift.setDirection(DcMotorSimple.Direction.FORWARD);

        initialPosition = (int) (motorLift.getCurrentPosition());
        rotations = 12;
        directionArm = -1;
        targetPosition = (int) (motorLift.getCurrentPosition() + (directionArm * rotations * COUNTS_PER_MOTOR_REV));

        //utils class initializer
        teamUtils = new Util(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Play started
        runtime.reset();

        while (opModeIsActive()) {
            // run this loop until the end of the match (driver presses stop)
            teamUtils.drivebyDistance(0.5, 0.0, 0.0, 32);
            teamUtils.drivebyDistance(0.5, 0.0, 0.0, 45);
            teamUtils.drivebyDistance(0.0, 0.5, 0.0, 12);

        }
    }
}


