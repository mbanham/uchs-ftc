package org.firstinspires.ftc.team11288;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;


@Autonomous(name = "Blue_Right_Center_Park", group = "Linear Opmode")
//@Disabled                            // Comment this out to add to the opmode list
public class Blue_Right_Center_Park extends LinearOpMode {
    //initialize these variables, override them in the constructor

    /* Declare OpMode members. */
    //wheels
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor motorLift;

    private UtilHolonomic teamUtils;


    //    private elbow             = null;
//    private Servo wrist       = null;
    private Servo claw = null;
    private Servo platform = null;


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
        platform.setPosition(1);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLift.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(STOP_AND_RESET_ENCODER);

        //utils class initializer
        teamUtils = new UtilHolonomic(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, telemetry);
        // teamUtils.InitExtraSensors(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Play started

        boolean stepsCompleted = false;
        runtime.reset();
        while (opModeIsActive()) {
            if (!stepsCompleted) {
                stepsCompleted = true;
                // run this loop until the end of the match (driver presses stop)

                try {
                    Thread.sleep(15000);
                } catch(Exception e) {}


                teamUtils.drivebyDistance(0.85, 0.0, 3);//drive away from wall
                teamUtils.drivebyDistance(0.8, 0.0, 24);//drive towards center
                teamUtils.drivebyDistance(0.0, 0.8, 24);//drive towards bridge
                claw.setPosition(1);
                teamUtils.stopWheelsSpeedMode();
                requestOpModeStop();

            }
        }
    }
}


