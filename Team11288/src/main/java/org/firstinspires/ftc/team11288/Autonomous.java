package org.firstinspires.ftc.team11288;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Autonomous extends OpMode {

    DcMotor mfl, mfr, mbl, mbr;

    @Override
    public void init() {
        mfl = hardwareMap.dcMotor.get("motor front left");
        mfr = hardwareMap.dcMotor.get("motor front right");
        mbl = hardwareMap.dcMotor.get("motor back left");
        mbr = hardwareMap.dcMotor.get("motor back right");
    }

    @Override
    public void loop() {

    }
}
