package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Asynchronous state machine for auto (see https://gm0.org/en/latest/docs/software/finite-state-machines.html)
@Autonomous(group = "advanced")
public class FFAutoBlue extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        SPACE_FORWARD,
        GO_TO_CAROUSEL,
        SPIN_CAROUSEL,
        GO_TO_STORAGE,
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-36, 61, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize carousel spinner
        DcMotor carouselSpinner = hardwareMap.dcMotor.get("carousel spinner");

        // Initialize SampleMecanumDrive & set initial pose
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .forward(4)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(-60, 60, 0))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineTo(new Vector2d(-70, 27))
                .build();

        // For waiting
        ElapsedTime waitTimer = new ElapsedTime();


        waitForStart();
        if (isStopRequested()) return;

        // trajectory1: Move forward to allow turning
        currentState = State.SPACE_FORWARD;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {

            // State machine
            switch (currentState) {

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