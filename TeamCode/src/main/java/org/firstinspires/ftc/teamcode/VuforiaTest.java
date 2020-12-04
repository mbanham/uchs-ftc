
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "VuforiaTest", group = "Linear Opmode")
//@Disabled                            // Comment this out to add to the opmode list
public class VuforiaTest extends LinearOpMode {

    int i = 1;

    @Override
    public void runOpMode() {

        VuforiaUtil.Initialize(hardwareMap, telemetry);
        VuforiaUtil.startTFOD();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("info", "%d", i++);

            //Play started
            ArrayList<ViewObject> viewObjects = VuforiaUtil.getObjectsInFrame();
            for (int i = 0; i < viewObjects.size(); i++) {
                ViewObject viewObject = viewObjects.get(i);
                //assumes y is up
                telemetry.addData("Status", "object detected " + Math.round(Math.sqrt(Math.pow(viewObject.transform.z, 2) + Math.pow(viewObject.transform.x, 2))) +
                        "\n distance away at an angle of " + Math.atan2(viewObject.transform.x, viewObject.transform.z));
            }
            if(viewObjects.size()==0){
                telemetry.addData("Status", "no object detected");

            }
            telemetry.update();
        }
    }
}