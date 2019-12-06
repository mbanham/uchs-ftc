package org.firstinspires.ftc.team11288;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

public class Autonomous extends OpMode {

    DcMotor mfl, mfr, mbl, mbr;


    @Override
    public void init() {



        mfl = hardwareMap.dcMotor.get("motor front left");
        mfr = hardwareMap.dcMotor.get("motor front right");
        mbl = hardwareMap.dcMotor.get("motor back left");
        mbr = hardwareMap.dcMotor.get("motor back right");
        /*teamUtils.drivebyDistance(-0.5, 0.0, 0.0, 3);
        teamUtils.drivebyDistance(0.0, 0.5, 0.0, 60);
        platform.setPosition(0);
        teamUtils.drivebyDistance(0.5, 0, 0.0, 28);
        platform.setPosition(1);
        teamUtils.drivebyDistance(-0.5, 0.0, 0.0, 30);
        platform.setPosition(0);
        teamUtils.drivebyDistance(0.0, -0.5, 0.0, 60);*/
    }

    @Override
    public void loop() {

    }
}
