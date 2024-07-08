package org.firstinspires.ftc.teamcode.drive.path;
import org.opencv.core.Point;
public class Robot {
    private Point location;

    public Robot(Point location){
        this.location = location;
    }
    public Point getLocation(){
        return location;
    }
    public void setPosition(Point target){
        location = target;
    }
    public void updateControl() {
        System.out.println("Simulating robot movement...");
        System.out.println("Current Location: " + location);
    }
}
