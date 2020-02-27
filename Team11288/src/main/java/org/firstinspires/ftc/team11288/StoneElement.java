package org.firstinspires.ftc.team11288;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;;

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
        name = recog.getLabel();
        size = new Size(recog.getWidth(), recog.getHeight());
        screen_size = new Size(recog.getImageWidth(), recog.getImageHeight());
        screen_center = new Point(recog.getImageWidth()/2, recog.getImageHeight()/2);
        center = new Point(recog.getLeft() + size.width/2, recog.getBottom() + size.height/2);
        bottom_left = new Point(recog.getLeft(), recog.getBottom());
        bottom_right = new Point(recog.getRight(), recog.getBottom());
        top_left = new Point(recog.getLeft(), recog.getTop());
        top_right = new Point(recog.getRight(), recog.getTop());
    }
    public boolean nearCenter(int pixel_threshold){
        double x = Math.abs(center.x - screen_center.x);
        double y = Math.abs(center.y - screen_center.y);
        double dist = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        return dist < pixel_threshold;
    }
}
