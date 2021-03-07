package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.AI.VuforiaInitializer;
import org.firstinspires.ftc.teamcode.Abstracts.Point;
import org.firstinspires.ftc.teamcode.Abstracts.RingReturnObject;
import org.firstinspires.ftc.teamcode.Utilities.DeviceManager;
import org.firstinspires.ftc.teamcode.Utilities.RingCountDetection;
import org.firstinspires.ftc.teamcode.Utilities.RobotMovementIntegrated;
import org.firstinspires.ftc.teamcode.Utilities.UtilHolonomic;

import static org.firstinspires.ftc.teamcode.AI.VuforiaInitializer.VUFORIA_KEY;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousProcedureBlue", group = "Linear Opmode")
//@Disabled                            // Comment this out to add to the opmode list
public class AutonomousDriveBlue extends LinearOpMode {

    DcMotor mfl, mfr, mbl, mbr;
    private UtilHolonomic teamUtils;

    Point endPointA = new Point(12, 84);//a
    Point endPointB = new Point(36, 108);//b
    Point endPointC = new Point(12, 132);//c

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        System.out.println(hardwareMap.toString());

        VuforiaInitializer.InitializeVuforia(hardwareMap, VuforiaInitializer.Modules.ObjectDetection, VuforiaInitializer.Modules.RobotTransformDetection);
        DeviceManager.MapDriveMotors(hardwareMap);

        waitForStart();
        VuforiaInitializer.BeginTracking();
        //one time run
        RingReturnObject ringreturn = null;
        while (!RingCountDetection.GetRingCountCheck()) {

        }
        ringreturn = RingCountDetection.GetRingCount();
        if(ringreturn==null){
            requestOpModeStop();
        }else{
            switch (ringreturn.ring_count){
                case 1:
                    //Do procedure for one ring
                    RobotMovementIntegrated.MoveRobotToLocation(endPointA.x, endPointA.y);//A
                    break;
                case 2:
                    //Do procedure for two rings
                    RobotMovementIntegrated.MoveRobotToLocation(endPointB.x, endPointB.y);//B
                    break;
                case 3:
                    //Do procedure for three rings
                    RobotMovementIntegrated.MoveRobotToLocation(endPointC.x, endPointC.y);//C
                    break;
            }
        }
        VuforiaInitializer.EndTracking();
    }
}
