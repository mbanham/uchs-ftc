package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class DeviceManager {
    public static DcMotor motorFrontRight;
    public static DcMotor motorFrontLeft;
    public static DcMotor motorBackRight;
    public static DcMotor motorBackLeft;
    public static DcMotor launcherMotor;
    public static DcMotor conveyorMotor;
    public static WebcamName webcamName;

    public static void MapDriveMotors(){
        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");
    }
    public static void MapLauncherMotors(){
        launcherMotor = hardwareMap.dcMotor.get("motor front right");
    }
    public static void MapConveyorMotors(){
        conveyorMotor = hardwareMap.dcMotor.get("motor front right");
    }
    public static void MapWebcam(){
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }
}
