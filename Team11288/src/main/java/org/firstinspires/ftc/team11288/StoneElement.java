package org.firstinspires.ftc.team11288;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class StoneElement {
    public Size size;
    public Size screen_size;
    public Point bottom_left;
    public Point bottom_right;
    public Point top_left;
    public Point top_right;
    public Point screen_center;
    public Point center;
    public String name;
    public StoneElement(Recognition recog){
        //origin is bottom left
        this.name = recog.getLabel();
        this.size = new Size(recog.getWidth(), recog.getHeight());
        this.screen_size = new Size(recog.getImageWidth(), recog.getImageHeight());
        this.screen_center = new Point(recog.getImageWidth()/2, recog.getImageHeight()/2);
        this.center = new Point(recog.getLeft() + size.width/2, recog.getBottom() + size.height/2);
        this.bottom_left = new Point(recog.getLeft(), recog.getBottom());
        this.bottom_right = new Point(recog.getRight(), recog.getBottom());
        this.top_left = new Point(recog.getLeft(), recog.getTop());
        this.top_right = new Point(recog.getRight(), recog.getTop());
    }
    public boolean nearCenter(int pixel_threshold){
        double x = Math.abs(center.x - screen_center.x);
        double y = Math.abs(center.y - screen_center.y);
        double dist = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        return dist < pixel_threshold;
    }
}
