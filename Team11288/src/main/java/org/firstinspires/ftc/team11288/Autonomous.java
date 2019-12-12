package org.firstinspires.ftc.team11288;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Linear Opmode")
//@Disabled                            // Comment this out to add to the opmode list

public class Autonomous extends LinearOpMode {

    DcMotor mfl, mfr, mbl, mbr;
    private Util teamUtils;
    private ColorSensor colorSensor;
    private float[] hsvValues;
    private int SCALE_FACTOR = 255;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        // hsvValues is an array that will hold the hue, saturation, and value information.
        hsvValues = new float[] {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        float values[] = hsvValues;
        waitForStart();
        while(opModeIsActive()) {
            colorSensor.enableLed(true);
            //for those motors that should be busy (power!=0) wait until they are done
            //reaching target position before returning from this function.

            Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                    (int) (colorSensor.green() * SCALE_FACTOR),
                    (int) (colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
        }
    }
}
