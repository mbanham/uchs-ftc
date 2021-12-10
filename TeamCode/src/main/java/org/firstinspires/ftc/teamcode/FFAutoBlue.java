package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.UtilHolonomic;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;


@Autonomous(name = "Freight Frenzy Auto: Blue", group = "Linear Opmode")
@Disabled                            // Comment this out to add to the opmode list
public class FFAutoBlue extends LinearOpMode {
    //initialize these variables, override them in the constructor

    /* Declare OpMode members. */
    //wheels
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    private DcMotor carouselSpinner;

    private UtilHolonomic teamUtils;

    private ElapsedTime runtime;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

        //initialize wheels
        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");
        carouselSpinner = hardwareMap.dcMotor.get("carousel spinner");
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        //utils class initializer
        teamUtils = new UtilHolonomic(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Play started

        int startTime = (int) runtime.seconds();

        while (opModeIsActive()) {
                // run this loop until the end of the match (driver presses stop)
            int currentTime = (int) runtime.seconds();

            // spin motor every other second
            carouselSpinner.setPower(currentTime % 2 == 0 ? 0.4 : 0.0);

            // start after 12s
            if(currentTime <= startTime + 12) {
                continue;
            }
            teamUtils.DriveByDistance(1, 1, 12);

            }
        }
}


