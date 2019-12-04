package org.firstinspires.ftc.team11288;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

public class Autonomous extends OpMode {

    DcMotor mfl, mfr, mbl, mbr;
    ArrayList<ArrayList<String>> programs = new ArrayList<ArrayList<String>>();

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
    public void InitProgram(ArrayList<String> commands){
        //programs.add()
    }
}
