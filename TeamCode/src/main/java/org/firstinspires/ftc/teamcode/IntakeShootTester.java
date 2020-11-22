// Team8380_Teleop
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This file provides Teleop tests for the Team8380 robot.
 *
 * The code is structured as an Iterative OpMode
 *
 * Test for intake motor and shooting motor
 *
 *
 */
//@Disabled
@TeleOp(name = "IntakeShootTester", group = "Teleop")
public class IntakeShootTester extends OpMode {
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double multiplier = 1;
    /* Declare OpMode members. */
    private DcMotor motorIntake;
    private DcMotor motorShooter;
    private Servo servoIntake = null;

    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void init() {
        // Initialize the hardware variables.
        // Send telemetry message to signify robot waiting
        telemetry.addData("Welcome", "Hello Driver");
        //initialize
        motorIntake = hardwareMap.dcMotor.get("motor intake");
        motorShooter = hardwareMap.dcMotor.get("motor shooter");
        servoIntake = hardwareMap.servo.get("intake servo");
        servoIntake.setPosition(0);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorShooter.setDirection(DcMotorSimple.Direction.FORWARD);
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

    @Override
    public void loop() {
        //intake arm
        if (gamepad1.dpad_down) {
            servoIntake.setPosition(1);
            telemetry.addData("MyActivity", "ClawPosition=1");
            telemetry.update();
        }
        if (gamepad1.dpad_up) {
            servoIntake.setPosition(0);
            telemetry.addData("MyActivity", "ClawPosition=0");
            telemetry.update();
        }

        //intake
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setPower(Range.clip(gamepad1.right_trigger, 0, 1.0));

        //shooter
        motorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setPower(Range.clip(gamepad1.left_trigger, 0, 1.0));

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
