package org.firstinspires.ftc.teamcode.drive.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.posePID2.NewDT;

import java.util.ArrayList;
import java.util.Collections;

public class PurePursuit {
    private NewDT drive;
    private ArrayList<Pose2d> wayPoints = new ArrayList<>();
    private double moveRadius, headingRadius;
    private Pose2d lastTranslatePoint = new Pose2d(0,0);
    private Pose2d lastHeadingPoint = new Pose2d(0,0);
    private double movePower;
    private double headingOffset;
    private boolean velextra = false;
    private boolean trigger = true;
    Pose2d followDrive, followHeading;
    private double pathLength =0;

    public PurePursuit(NewDT drive, double moveRadius, double headingRadius, double movePower){
        this.drive = drive;
        this.moveRadius = moveRadius;
        this.headingRadius = headingRadius;
        drive.setFollowRadius(moveRadius);

        drive.setOn(true);
        this.movePower = movePower;

    }

    public void followPath(boolean newPath, ArrayList<Pose2d> wayPoints, double headingOffset){
        this.headingOffset = headingOffset;
        if(newPath && trigger){
            PurePursuitUtil.updateMoveSegment(1);
            PurePursuitUtil.updateHeadingSegment(1);
            trigger = false;
        }
        this.wayPoints = wayPoints;
        lastTranslatePoint = wayPoints.get(0);
        lastHeadingPoint = wayPoints.get(0);

        if(velextra){
            followDrive = PurePursuitUtil.followMe(wayPoints, drive.getFuturePos(500), moveRadius, lastTranslatePoint, false);
            lastTranslatePoint = followDrive;

            //true for heading

            followHeading = PurePursuitUtil.followMe(wayPoints, drive.getFuturePos(500), headingRadius, lastHeadingPoint, true);
        }
        else{
            followDrive = PurePursuitUtil.followMe(wayPoints, drive.getLocation(), moveRadius, lastTranslatePoint, false);
            lastTranslatePoint = followDrive;

            //true for heading

            followHeading = PurePursuitUtil.followMe(wayPoints, drive.getLocation(), headingRadius, lastHeadingPoint, true);
        }
        lastHeadingPoint = followHeading;

        drive.lineTo(followDrive.getX(), followDrive.getY(),drive.toPoint(drive.getX(), drive.getY(), drive.getR(), followHeading.getX(), followHeading.getY())+headingOffset);
        pathLength = Math.hypot((wayPoints.get(PurePursuitUtil.getMoveSegment()).getX()-drive.getX()), (wayPoints.get(PurePursuitUtil.getMoveSegment()).getY()-drive.getY()));
        for(int i=PurePursuitUtil.getMoveSegment()+1;i<wayPoints.size();i++){
            pathLength += Math.hypot((wayPoints.get(i-1).getX()-wayPoints.get(i).getX()), (wayPoints.get(i-1).getY()-wayPoints.get(i).getY()));
        }
        drive.setPathLength(pathLength);
        drive.setMaxPower(movePower);
        drive.update();
    }
    public Pose2d getFollowHeading(){
        return followHeading;
    }
    public Pose2d getFollowDrive(){
        return followDrive;
    }
    public void setMovePower(double movePower){
        this.movePower = movePower;
    }
    public double getPathLength(){
        return pathLength;
    }
}
