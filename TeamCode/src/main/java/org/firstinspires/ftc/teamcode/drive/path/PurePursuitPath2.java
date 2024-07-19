package org.firstinspires.ftc.teamcode.drive.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
import org.firstinspires.ftc.teamcode.drive.posePID2.NewDT;

import java.util.ArrayList;
import java.util.Collections;

public class PurePursuitPath2 {
    private ArrayList<Pose2d> wayPoints = new ArrayList<>();
    private DT drive;
    private double moveRadius, headingRadius;
    private Pose2d lastTranslatePoint = new Pose2d(0,0);
    private Pose2d lastHeadingPoint = new Pose2d(0,0);
    private double movePower;
    private double headingOffset;
    private boolean velextra = false;
    public PurePursuitPath2(DT drive, double moveRadius, double headingRadius, Pose2d... ws){
        this.drive = drive;
        Collections.addAll(wayPoints, ws);
        this.moveRadius = moveRadius;
        this.headingRadius = headingRadius;
    }

    public void init(double movePower, double headingOffset){
        drive.setFollowRadius(moveRadius);
        PurePursuitUtil.updateSegment(1);
        PurePursuitUtil.updateEnding(false);
        drive.setOn(true);
        lastTranslatePoint = wayPoints.get(0);
        lastHeadingPoint = wayPoints.get(0);
        this.movePower = movePower;
        this.headingOffset = headingOffset;
    }
    public void setVelExtra(boolean current){
        velextra = current;
    }
    public boolean getVelExtra(){
        return velextra;
    }
    public void setMovePower(double movePower){
        this.movePower = movePower;
    }
    public void update() {
        Pose2d followDrive, followHeading;
        //false for urm non heading ig
        //getFuturePos for vel extrapolation
        if(velextra){
            followDrive = PurePursuitUtil.followMe(wayPoints, drive.getFuturePos(500), moveRadius, lastTranslatePoint, false);
            lastTranslatePoint = followDrive;

            //true for heading

            followHeading = PurePursuitUtil.followMe(wayPoints, drive.getFuturePos(500), headingRadius, lastHeadingPoint, true);
            lastHeadingPoint = followHeading;
        }
        else{
            followDrive = PurePursuitUtil.followMe(wayPoints, drive.getLocation(), moveRadius, lastTranslatePoint, false);
            lastTranslatePoint = followDrive;

            //true for heading

            followHeading = PurePursuitUtil.followMe(wayPoints, drive.getLocation(), headingRadius, lastHeadingPoint, true);
            lastHeadingPoint = followHeading;
        }
        if(PurePursuitUtil.getEnding()) {
            drive.setPathEndHold(true);
            drive.lineTo(followDrive.getX(), followDrive.getY(),Math.toRadians(-180));
        }else{
            drive.lineTo(followDrive.getX(), followDrive.getY(),drive.toPoint(drive.getX(), drive.getY(), drive.getR(), followHeading.getX(), followHeading.getY() + headingOffset));
        }
        drive.setMaxPower(movePower);
        drive.update();
    }
    public boolean isFinished() {
        // Implement logic to determine if the path following is finished
        return PurePursuitUtil.getEnding(); // Placeholder logic
    }
}

