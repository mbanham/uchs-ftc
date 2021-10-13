package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.UtilMecanum;

@Autonomous(name = "Aleksa Autonomous", group = "LinearOpMode")
public class AutonomousNew extends LinearOpMode {

    private static DcMotor motorFrontRight;
    private static DcMotor motorFrontLeft;
    private static DcMotor motorBackRight;
    private static DcMotor motorBackLeft;
    private static DcMotor motorLauncher;
    private static Servo wobbler;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");
        wobbler = hardwareMap.servo.get("wobble servo");
        motorLauncher = hardwareMap.dcMotor.get("motor launcher");

        UtilMecanum.init(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, wobbler);

        wobbler.setPosition(0.7);

        UtilMecanum.shiftTarget(0, 36, 0);

        waitForStart();

        while (opModeIsActive()) {
            UtilMecanum.update(getRuntime());
            if (UtilMecanum.inLeftY == 0) wobbler.setPosition(0.3);
//            if(UtilMecanum.inLeftR == 0) motorLauncher.setPower(1);
        }
    }
}
