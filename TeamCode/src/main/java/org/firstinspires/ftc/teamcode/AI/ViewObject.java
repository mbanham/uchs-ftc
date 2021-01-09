package org.firstinspires.ftc.teamcode.AI;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class Transform{
    float x;
    float y;
    float z;
}
class Rotation{
    float rx;
    float ry;
    float rz;
}

public class ViewObject {
    String type;
    Transform transform = new Transform();
    Rotation rotation = new Rotation();

    public ViewObject(String type, VectorF transform, Orientation rot){
        this.type = type;
        this.transform.x = transform.get(0);
        this.transform.y = transform.get(1);
        this.transform.z = transform.get(2);

        // Extract the rotational components of the target relative to the robot
        this.rotation.rx = rot.firstAngle;
        this.rotation.ry = rot.secondAngle;
        this.rotation.rz = rot.thirdAngle;
    }
}
