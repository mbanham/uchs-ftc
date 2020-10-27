package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;


@Autonomous(name = "Ref_Right_Platf_Center", group = "Linear Opmode")
//@Disabled                            // Comment this out to add to the opmode list
public class Ref_Right_Platf_Center extends LinearOpMode {
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
        //platform = hardwareMap.servo.get("platform servo");
        claw.setPosition(0);
        //platform.setPosition(1);
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
        teamUtils.InitExtraSensors(hardwareMap);
        teamUtils.InitPlatform(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Play started

        boolean stepsCompleted = false;

        while (opModeIsActive()) {
            if (!stepsCompleted) {
                stepsCompleted = true;
                // run this loop until the end of the match (driver presses stop)
                teamUtils.drivebyDistance(0.85, 0.0, UtilHolonomic.ROBOT_WALL_CLEARANCE);//drive away from wall
                teamUtils.drivebyDistance(0.0, -0.85, UtilHolonomic.MARKER_A_TO_PLATFORM_CENTER);//drive towards corner
                teamUtils.drivebyDistance(0.85, 0, UtilHolonomic.EDGE_TO_PLATFORM_CLEARANCE);//drive to base plate

                teamUtils.GrabPlaform(true);

                //drive back to corner
                teamUtils.GrabPlaform(false);
                sleep(800);
                teamUtils.drivebyDistance(-0.85, 0, UtilHolonomic.WALL_ROBOT_TO_EDGE_LOAD, UtilHolonomic.ROBOT_WALL_CLEARANCE);//drive towards corner
                teamUtils.GrabPlaform(true);
                sleep(800);

                teamUtils.drivebyDistance(0.0, 0.85, 0.85 * (0.66 * UtilHolonomic.BRIDGE_TO_PLATFORM_CENTER));//drive up to park at wall
                teamUtils.drivebyDistance(0.85, 0, UtilHolonomic.WALL_TO_CENTER);//drive towards corner
                teamUtils.drivebyDistance(0.0, 0.85, 0.85 * (0.33 * UtilHolonomic.BRIDGE_TO_PLATFORM_CENTER));//drive up to park at wall
                teamUtils.stopWheelsSpeedMode();
                claw.setPosition(1);

                try {
                    Thread.sleep(3000);
                } catch (Exception e) {
                }

                requestOpModeStop();
            }
        }
    }
}


