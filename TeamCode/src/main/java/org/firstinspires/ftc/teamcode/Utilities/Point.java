package org.firstinspires.ftc.teamcode.Utilities;

public class Point {
    public float x;
    public float y;

    public Point(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public double distanceToPoint(Point p) {
        double x = p.x - this.x;
        double y = p.y - this.y;
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }
}
