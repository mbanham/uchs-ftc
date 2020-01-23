// Team11288_Teleop
package org.firstinspires.ftc.team11288;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

/*
 * This file provides Teleop driving for the Team11288 TeleopDrive drive robot.
 * The code is structured as an Iterative OpMode
 *
 * Assumes claw with arm having shoulder motor, elbow servo and wrist servo - all having 180deg servos
 *
 */
//@Disabled
@TeleOp(name="TeleopDrive", group="Teleop")
public class TestDriveByDistance extends OpMode{

    /* Declare OpMode members. */
  //wheels
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor motorArm;
    private DcMotor motorLift;

    private UtilHolonomic teamUtils;

    private Servo claw        = null;
    private Servo platform    = null;



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
    double multiplier = 1;
    double arm_multiplier = 1;
    @Override
    public void loop() {

        //platformArm
       if(Range.clip(gamepad1.right_trigger, 0.4, 1) > 0.41){
            multiplier=1-Range.clip(gamepad1.right_trigger, 0, 0.4);
        }else
        {
            multiplier = 1;
        }
        //#region PLATFORM_GRABBER
        if (gamepad1.a) {
            //down
            platform.setPosition(0);
            telemetry.addData("MyActivity", "ServoPosition=0");
            telemetry.update();
        } else if (gamepad1.y) {
            //up
            platform.setPosition(1);
            telemetry.addData("MyActivity", "ServoPosition=1");
            telemetry.update();
        }
        //#endregion



            //dpad control 12 inch
            double x_int = 0;
            double y_int = 0;
            if (gamepad1.dpad_up)
                y_int = 0.8;
            if(gamepad1.dpad_down)
                y_int = -0.8;
            if(gamepad1.dpad_left)
                x_int = 0.8;
            if(gamepad1.dpad_right)
                x_int = -0.8;
            if(x_int != 0 || y_int != 0) {
                teamUtils.drivebyDistance(x_int, y_int, 10, "inch");
            }


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
