package org.firstinspires.ftc.teamcode;

public class Point {
    public double x;
    public double y;
    public double x_max;
    public double y_max;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(double x, double y, double x_max, double y_max) {
        this.x = x;
        this.y = y;
        this.x_max = x_max;
        this.y_max = y_max;
    }

    public double percent_X() {
        if (x_max == 0)
            return -1;
        return x / x_max;
    }

    public double percent_Y() {
        if (y_max == 0)
            return -1;
        return y / y_max;
    }

    public double distanceToPoint(Point p) {
        double x = p.x - this.x;
        double y = p.y - this.y;
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public double distanceToPointX(Point p) {
        double x = p.x - this.x;
        return x;
    }

    public double distanceToPointY(Point p) {
        double y = p.y - this.y;
        return y;
    }
}
