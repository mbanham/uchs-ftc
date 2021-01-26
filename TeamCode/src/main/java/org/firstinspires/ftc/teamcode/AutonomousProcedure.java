package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousProcedure", group = "Linear Opmode")
//@Disabled                            // Comment this out to add to the opmode list
public class AutonomousProcedure extends LinearOpMode {

    DcMotor mfl, mfr, mbl, mbr;
    private UtilHolonomic teamUtils;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RingCountDetection.Initialize(hardwareMap);

        waitForStart();
        //one time run
        RingReturnObject ringreturn = null;
        while (opModeIsActive()) {
            ringreturn = RingCountDetection.GetRingCount();
            //if method does not return null and at least 10% of the maximum iterations has been performed
            if (ringreturn.ring_count != -1 && (ringreturn.listsize / ringreturn.maxlistsize) > 0.1) {
                break;
            }
        }
        if(ringreturn==null){
            requestOpModeStop();
        }else{
            switch (ringreturn.ring_count){
                case 1:
                    //Do procedure for one ring
                    RobotMovementIntegrated.MoveRobotToLocationDualSensor(132, 84);//A
                    break;
                case 2:
                    //Do procedure for two rings
                    RobotMovementIntegrated.MoveRobotToLocationDualSensor(108,108);//B
                    break;
                case 3:
                    //Do procedure for three rings
                    RobotMovementIntegrated.MoveRobotToLocationDualSensor(132,132);//C
                    break;
            }
        }
    }
}
