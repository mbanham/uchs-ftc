package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Utilities.DeviceManager.motorBackLeft;
import static org.firstinspires.ftc.teamcode.Utilities.DeviceManager.motorBackRight;
import static org.firstinspires.ftc.teamcode.Utilities.DeviceManager.motorFrontLeft;
import static org.firstinspires.ftc.teamcode.Utilities.DeviceManager.motorFrontRight;

public class UtilMecanum {
    static float IN_PER_SEC = 6.5f;
    static double lastTime;

    private static DcMotor motorFrontRight;
    private static DcMotor motorFrontLeft;
    private static DcMotor motorBackRight;
    private static DcMotor motorBackLeft;
    private static Servo wobbler;

    public static void init(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, Servo w) {
        motorFrontRight = fr;
        motorFrontLeft = fl;
        motorBackRight = br;
        motorBackLeft = bl;
        wobbler = w;
        lastTime = 0;
    }

    static final double motorPower = 0.7;
    public static double inLeftY, inLeftX, inLeftR, change;

    public static void shiftTarget(double x, double y, double r) {
        inLeftX += x;
        inLeftY += y;
        inLeftR += r*7.5;
    }

    public static void update(double runtime) {
        double timeChange = 0;
        if(lastTime != 0) {
            timeChange = runtime - lastTime;
        }

        if(inLeftY != 0) {
            change = timeChange*IN_PER_SEC;

            if(Math.abs(change) >= Math.abs(inLeftY)) inLeftY = 0;
            else inLeftY -= change;

            if(inLeftY > 0) {
                updateMotors(0,-1,0);
            } else if(inLeftY < 0) {
                updateMotors(0,1,0);
            }
        } else if(inLeftX != 0) {
            change = timeChange*IN_PER_SEC;

            if(Math.abs(change) >= Math.abs(inLeftY)) inLeftY = 0;
            else inLeftY -= change;

            if(inLeftY > 0) {
                updateMotors(0,1,0);
            } else if(inLeftY < 0) {
                updateMotors(0,-1,0);
            }
        } else if(inLeftR != 0) {
            change = timeChange*IN_PER_SEC;

            if(Math.abs(change) >= Math.abs(inLeftR)) inLeftR = 0;
            else inLeftR -= change;

            if(inLeftR > 0) {
                updateMotors(0,0,1);
            } else if(inLeftR < 0) {
                updateMotors(0,0,-1);
            }
        } else {
            updateMotors(0,0,0);
        }

        lastTime = runtime;
    }

    private static void updateMotors(double stickx, double sticky, double rightX) {
        double r = Math.hypot(stickx, sticky);
        double robotAngle = Math.atan2(sticky, -stickx) - Math.PI / 4;

        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = -r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = -r * Math.cos(robotAngle) - rightX;

        double FrontRight = Range.clip(v2 * motorPower, -1, 1);
        double FrontLeft = Range.clip(v1 * motorPower, -1, 1);
        double BackLeft = Range.clip(v3 * motorPower, -1, 1);
        double BackRight = Range.clip(v4 * motorPower, -1, 1);

        // write the values to the motors
        setWheelsToSpeedMode();
        motorFrontRight.setPower(FrontRight);
        motorFrontLeft.setPower(FrontLeft);
        motorBackLeft.setPower(BackLeft);
        motorBackRight.setPower(BackRight);
    }

    public static void setWheelsToSpeedMode() {

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}