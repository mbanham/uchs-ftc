package org.firstinspires.ftc.team11288;

public class Point {
    public double x;
    public double y;
    public double x_max;
    public double y_max;

    public double percent_X(){
        if(x_max == 0)
            return -1;
        return x / x_max;
    }
    public double percent_Y(){
        if(y_max == 0)
            return -1;
        return y / y_max;
    }

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }
    public Point(double x, double y, double x_max, double y_max){
        this.x = x;
        this.y = y;
        this.x_max = x_max;
        this.y_max = y_max;
    }
}
