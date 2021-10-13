// Team11288_Teleop
package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.UtilHolonomic;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

/*
 * This file provides Teleop driving for the Team11288 TeleopDrive drive robot.
 * The code is structured as an Iterative OpMode
 *
 * Assumes claw with arm having shoulder motor, elbow servo and wrist servo - all having 180deg servos
 *
 */
//@Disabled
@TeleOp(name = "TestDriveByDistance", group = "Teleop")
@Disabled
public class TestDriveByDistance extends OpMode {

    double inches = 10.0;
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double multiplier = 1;
    double arm_multiplier = 1;
    /* Declare OpMode members. */
    //wheels
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor motorLift;
    private UtilHolonomic teamUtils;
    private Servo claw = null;
    private Servo platform = null;

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
        platform = hardwareMap.servo.get("platform_right");
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
        teamUtils.InitExtraSensors(hardwareMap);
        inches = 10.0;

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

        //platformArm
        if (Range.clip(gamepad1.right_trigger, 0.4, 1) > 0.41) {
            multiplier = 1 - Range.clip(gamepad1.right_trigger, 0, 0.4);
        } else {
            multiplier = 1;
        }


        //set how far to drive
        if (gamepad1.left_bumper) {
            inches -= 1.0;
            inches = Range.clip(inches, 0.0, 60.0);
            telemetry.addData("Distance", "val=" + inches);
            telemetry.update();
            while (gamepad1.left_bumper) {
                try {
                    Thread.sleep(1000);
                } catch (Exception e) {
                }
            }
        }

        if (gamepad1.right_bumper) {
            inches += 1.0;
            inches = Range.clip(inches, 0.0, 60.0);
            telemetry.addData("Distance", "val=" + inches);
            telemetry.update();
            while (gamepad1.right_bumper) {
                try {
                    Thread.sleep(1000);
                } catch (Exception e) {
                }
            }
        }


        //dpad control drive inches in direction chosen
        double x_int = 0;
        double y_int = 0;
        if (gamepad1.dpad_up)
            y_int = 0.8;
        if (gamepad1.dpad_down)
            y_int = -0.8;
        if (gamepad1.dpad_left)
            x_int = 0.8;
        if (gamepad1.dpad_right)
            x_int = -0.8;
        if (x_int != 0 || y_int != 0) {
            teamUtils.DriveByDistance(x_int, y_int, inches, UtilHolonomic.DISTANCE_FROM_WALL);

        }
        telemetry.addData("Distance (inch)",
                String.format(Locale.US, "%.02f", UtilHolonomic.sensorDistance.getDistance(DistanceUnit.INCH)));
        telemetry.update();

        //claw
        if (gamepad2.right_bumper) {
            claw.setPosition(1);
            telemetry.addData("MyActivity", "ClawPosition=1");
            telemetry.update();
        }
        if (gamepad2.left_bumper) {
            claw.setPosition(0);
            telemetry.addData("MyActivity", "ClawPosition=0");
            telemetry.update();
        }


        //lift
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad2.dpad_down) {
            motorLift.setPower(-1);
        } else {
            if (gamepad2.dpad_up) {
                motorLift.setPower(1);

            } else {
                motorLift.setPower(0);
            }
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
