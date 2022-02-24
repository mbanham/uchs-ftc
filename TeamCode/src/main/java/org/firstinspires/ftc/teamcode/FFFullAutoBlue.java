package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Asynchronous state machine for auto (see https://gm0.org/en/latest/docs/software/finite-state-machines.html)
@Autonomous(group = "advanced")
public class FFFullAutoBlue extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        GO_TO_HUB,
        DEPOSIT_PRELOAD,
        SPACE_FORWARD,
        GO_TO_CAROUSEL,
        SPIN_CAROUSEL,
        GO_TO_STORAGE,
        IDLE            // Our bot will enter the IDLE state when done
    }

    // Heights of the different levels of the team hub
    int[] targetHeights = { 84, 160, 230 };

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-36, 61, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize carousel spinner
        DcMotor carouselSpinner = hardwareMap.dcMotor.get("carousel spinner");

        Servo intake = hardwareMap.servo.get("servo intake");

        DcMotor motorLift = hardwareMap.dcMotor.get("motor lift");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(0);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize SampleMecanumDrive & set initial pose
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory trajectory0 = drive.trajectoryBuilder(startPose)
                .strafeLeft(20)
                .build();

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .forward(4)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(-68, 58, Math.toRadians(-90)))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineTo(new Vector2d(-80, 29))
                .build();

        ElapsedTime waitTimer = new ElapsedTime();



        String cameraName = "Webcam 1";

        // Setup webcam helper
        WebcamName hardwareName = hardwareMap.get(WebcamName.class, cameraName);
        WebcamHelper webcam = new WebcamHelper(cameraName, hardwareName);

        // Turn on the webcam
        webcam.initiate();

        // Wait for autonomous to start
        waitForStart();

        // Get a single frame from the webcam
        Bitmap frame = webcam.getFrame();

        // Scale down image to 30% of the original size
        frame = webcam.scaleBitmap(frame, 0.3);

        // Get all green pixels with a threshold of 30
        int[] segments = webcam.getMajorityGreen(frame, 30);

        // Get the section with the most green
        int targetPosition = webcam.highestInArray(segments);

        // Save the image as a JPG
        //webcam.saveBitmap(frame);

        // Recycle the frame to save memory
        frame.recycle();

        // Stop the webcam
        webcam.stopCamera();



        if (isStopRequested()) return;

        // trajectory1: Move forward to allow turning
        currentState = State.GO_TO_HUB;
        drive.followTrajectoryAsync(trajectory0);

        while (opModeIsActive() && !isStopRequested()) {

            // State machine
            switch (currentState) {

                case GO_TO_HUB:

                    if(!drive.isBusy()) {
                        currentState = State.DEPOSIT_PRELOAD;
                        waitTimer.reset();
                    }
                    break;
                case DEPOSIT_PRELOAD:

                    motorLift.setPower(0.12);
                    motorLift.setTargetPosition(targetHeights[targetPosition]);

                    if(waitTimer.seconds() >= 2)
                        intake.setPosition(0.1);

                    if(waitTimer.milliseconds() >= 2300) {
                        intake.setPosition(0.5);
                        motorLift.setTargetPosition(0);
                    }


                    if(waitTimer.seconds() >= 7) {
                        motorLift.setPower(0);
                        currentState = State.SPACE_FORWARD;
                        drive.followTrajectoryAsync(trajectory1);
                    }
                    break;
                case SPACE_FORWARD:

                    // drive.isBusy == false when trajectory completed
                    if (!drive.isBusy()) {
                        currentState = State.GO_TO_CAROUSEL;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GO_TO_CAROUSEL:

                    if(!drive.isBusy()) {
                        currentState = State.SPIN_CAROUSEL;
                        waitTimer.reset();
                    }
                    break;
                case SPIN_CAROUSEL:

                    // spin carousel for 4 seconds
                    carouselSpinner.setPower(0.4);
                    drive.setMotorPowers(-0.1, -0.1, -0.1, -0.1);


                    if (waitTimer.seconds() >= 4) {
                        carouselSpinner.setPower(0.0);
                        drive.setMotorPowers(0.0, 0.0, 0.0, 0.0);
                        currentState = State.GO_TO_STORAGE;
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case GO_TO_STORAGE:

                    if(!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // Done, do nothing
                    break;
            }

            // We update drive continuously in the background, regardless of state
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

}
