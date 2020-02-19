package org.firstinspires.ftc.team11288;


import android.graphics.Color;
import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;


@Autonomous(name = "Camera_GrabStone", group = "Linear Opmode")
//@Disabled                            // Comment this out to add to the opmode list
public class Camera_GrabStone extends LinearOpMode {
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

    private UtilHolonomic teamUtils;


    //claw and arm
    static final double COUNTS_PER_MOTOR_REV = 1250.0; //HD Hex Motor (REV-41-1301) 40:1

    //    private elbow             = null;
//    private Servo wrist       = null;
    private Servo claw = null;
    private Servo platform = null;

    //TODO touch sensor
    DigitalChannel touchSensor;  // Hardware Device Object

    private int directionArm = 1;
    private int rotations = 12;


    private ElapsedTime runtime = new ElapsedTime();

    private UtilMain utilMain;

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
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLift.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(STOP_AND_RESET_ENCODER);

        //utils class initializer
        //teamUtils = new UtilMain(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, telemetry);
        //teamUtils.InitExtraSensors(hardwareMap);
        //teamUtils.InitVuforia(hardwareMap);

        utilMain = new UtilMain(telemetry);
        utilMain.InitVuforia(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Play started
        runtime.reset();

        List<Recognition> recog = utilMain.GetObjectsInFrame();
        for (Recognition r : recog){
            if(r.getLabel().equals(UtilMain.STONE)){
                double width = r.getImageWidth();
                double height = r.getImageHeight();
                double angle = r.estimateAngleToObject(AngleUnit.DEGREES);

                Point screen_center = new Point((int)(width / 2), (int)(height / 2));
                Point size = new Point((int)r.getWidth(), (int)r.getHeight());
                Point center = new Point((int)(r.getLeft() + size.x/2), (int)(r.getBottom() + size.y/2));
                Point bottom_left = new Point((int)r.getLeft(), (int)r.getBottom());
                Point bottom_right = new Point((int)r.getRight(), (int)r.getBottom());
                Point top_left = new Point((int)r.getLeft(), (int)r.getTop());
                Point top_right = new Point((int)r.getRight(), (int)r.getTop());
                
                telemetry.addData("debug", "x=%d;y=%d", center.x, center.y);
                telemetry.update();

                double threshold = 20;
                while(center.x >  screen_center.x + threshold || center.x < screen_center.x - threshold){
                    if(center.x >  screen_center.x + threshold) {//left too muchs
                        teamUtils.drivebySpeed(0.5, 0, 0);
                    }else if(center.x < screen_center.x - threshold){//right too much
                        teamUtils.drivebySpeed(-0.5, 0, 0);
                    }else{
                        teamUtils.stopWheelsSpeedMode();
                        requestOpModeStop();
                    }

                }



            }else{
                telemetry.addData("debug", "SKYSTONE");
                telemetry.update();
            }
        }
    }
}
