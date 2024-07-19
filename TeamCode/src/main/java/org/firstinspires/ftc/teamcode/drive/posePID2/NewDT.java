package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Math.T;

@Config
public class NewDT{

    private boolean forceStop = false;
    private SampleMecanumDrive drive;

    private DcMotorEx leftFront, rightRear, rightFront, leftRear;

    private double xTarget, yTarget, rTarget;
    private double xRn, yRn, rRn;
    private double deltaX, deltaY, deltaR;
    private boolean isAtTarget =false;



    public double turnVelocity;

    private boolean purePersuiting = false;

    private double xOut, yOut, rOut;
    private double twistedR, count, lastAngle;
    private double xPower, yPower;

    private boolean ending = false;

    private double maxPower = 1;
    private double followRadius = 0;
    private double flPower, frPower, blPower, brPower;

    private double normalize;
    private boolean on = true;
    ElapsedTime timer = new ElapsedTime();
    double currentTime = 0;
    double lastTime = 0;

    double xVelocity=0;
    double yVelocity = 0;
    public static double rkp=1.3;
    public static double xykp = 0.9;
    public static double xykd = 0;
    public static double rkd=0.0008;

    double deltaRRateOfChange = 0;
    double lastDeltaR = 0;

    double deltaXRateOfChange = 0;
    double lastDeltaX = 0;

    double deltaYRateOfChange = 0;
    double lastDeltaY = 0;

    double deltaTime = 0;


    public NewDT(HardwareMap hardwareMap, Pose2d startPose, ElapsedTime timer){
        this.timer = timer;
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.drive.setPoseEstimate(startPose);

        this.leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "br");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);

        this.xTarget = startPose.getX();
        this.yTarget = startPose.getY();
        this.rTarget = startPose.getHeading();
    }

    public void setPowers(double y, double x, double r){
        if(!forceStop){
            normalize = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
            flPower = (y+x-r);
            blPower = (y-x+r);
            brPower = (y+x+r);
            frPower = (y-x-r);
            leftFront.setPower((flPower/normalize));
            leftRear.setPower((blPower/normalize));
            rightRear.setPower((brPower/normalize));
            rightFront.setPower((frPower/normalize));
        }else{
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
    }
    public void update(){
        deltaTime = timeWrapper(timer.nanoseconds()) - lastTime;
        //change in time

        drive.updatePoseEstimate();
        xRn = drive.getPoseEstimate().getX();
        yRn = drive.getPoseEstimate().getY();
        rRn = drive.getPoseEstimate().getHeading();
        //position updates

        if (Math.abs(rRn - lastAngle) > M.PI) count += Math.signum(lastAngle - rRn);
        lastAngle = rRn;
        twistedR = count * (2 * M.PI) + rRn;
        //heading wrapper

        deltaR = -1 * (rTarget - twistedR);
        deltaX = xTarget - xRn;
        deltaY = -1 * (yTarget - yRn);
        //deltas


        deltaRRateOfChange = -1 * (deltaR - lastDeltaR) / (deltaTime);
        deltaYRateOfChange = -1 * (deltaY - lastDeltaY) / (deltaTime);
        deltaXRateOfChange = (deltaX - lastDeltaX) / (deltaTime);
        //rate of change of deltas (used for derivative in PID)

        xVelocity = deltaX / deltaTime;
        yVelocity = deltaY / deltaTime;
        turnVelocity = deltaR / deltaTime;
        //for velocity extrapolation, make sure they are in the right direction and right units



        //if statements start
        if(!ending){
            //pure persuit
            xOut = deltaX;
            yOut = deltaY;
            //literally gets deltas

            xOut/=followRadius;
            yOut/=followRadius;
            //normalizes between 0 and 1 which is what i think the *= follow radius stuff was
        } else {
            //pid
            xOut = (deltaX * xykp - deltaXRateOfChange * xykd);
            yOut = (deltaY * xykp - deltaYRateOfChange * xykd);
        }//if statements end



        rOut = (deltaR * rkp - deltaRRateOfChange * rkd);
        //heading power

        xPower = (xOut * T.cos(rRn) - yOut * T.sin(rRn));
        yPower = (xOut * T.sin(rRn) + yOut * T.cos(rRn));
        //rotation matrix
        if (Math.abs(xPower) > DTConstants.maxAxialPower)
            xPower = DTConstants.maxAxialPower * Math.signum(xPower);
        if (Math.abs(yPower) > DTConstants.maxAxialPower)
            yPower = DTConstants.maxAxialPower * Math.signum(yPower);
        if (Math.abs(rOut) > DTConstants.maxAngularPower)
            rOut = DTConstants.maxAngularPower * Math.signum(rOut);


        double zeroMoveAngle = Math.toRadians(25);
        double errorScale = 1 - (Math.abs(deltaR) / zeroMoveAngle);
        if (errorScale < 0) {
            errorScale = 0;
        }
        xPower *= errorScale;
        yPower *= errorScale;
        //if heading error is big, then drive slows down - make sure deltaR is not reversed cuz i changed sum stuff up



        setPowers(0, 0, -rOut);
        //actually sets power to the motor




        lastTime = timeWrapper(timer.nanoseconds());
        lastDeltaR = deltaR;
        lastDeltaX = deltaX;
        lastDeltaY = deltaY;
        //updating the last values

    }

    public void setPurePersuiting(boolean isPurePersuiting){
        purePersuiting = isPurePersuiting;
    }
    public double timeWrapper(double nano){
        return nano/1000000000;
    }
    public Pose2d getLocation(){
        return new Pose2d(xRn, yRn, rRn);
    }
    public void setXTarget(double x){
        this.xTarget = x;
    }
    public double getCurrentTime(){
        return currentTime;
    }
    public double getTimer(){
        return timer.nanoseconds();
    }
    public void setYTarget(double y){
        this.yTarget = y;
    }
    public void setRTarget(double r){
        this.rTarget = r;
    }
    public double getPowerX(){
        return xPower;
    }
    public double getPowerY(){
        return yPower;
    }
    public double getTurnVeloctiy(){return turnVelocity;}
    public double getPowerR(){
        return rOut;
    }
    public double getRawY(){
        return yOut;
    }
    public double getRawX(){
        return xOut;
    }
    public double getX(){
        return xRn;
    }
    public double getY(){
        return yRn;
    }
    public double getR(){
        return twistedR;
    }
    public double getYTarget(){
        return yTarget;
    }
    public double getXTarget(){
        return xTarget;
    }
    public double getRTarget(){
        return rTarget;
    }
    public double getUnTwistedR(){
        return rRn;
    }
    public double getDeltaX(){
        return deltaX;
    }
    public double getDeltaY(){
        return deltaY;
    }
    public double getDeltaR(){
        return deltaR;
    }
    //i think velocity is in ms, not 100% sure
    public Pose2d getFuturePos(int milliseconds){
        double r1 = xVelocity/turnVelocity;
        double r2 = yVelocity/turnVelocity;

        double relDeltaX = Math.sin(deltaR) * r1 - (1.0 - Math.cos(deltaR)) * r2;
        double relDeltaY = (1.0 - Math.cos(deltaR)) * r1 + Math.sin(deltaR) * r2;

        return new Pose2d(xRn+relDeltaX, yRn+relDeltaY);
    }
    public boolean isAtTarget(){
        return isAtTarget;
    }
    public boolean isAtTargetX(){
        return Math.abs(xRn - xTarget) < DTConstants.allowedAxialError;
    }
    public boolean isAtTargetY(){
        return Math.abs(yRn - yTarget) < DTConstants.allowedAxialError;
    }
    public boolean isAtTargetR(){
        return Math.abs(twistedR - rTarget) < DTConstants.allowedAngularError;
    }
    public double getTwistedR(){
        return 0;
    } //??
    public void setPoseEstimate(Pose2d pose){
        this.drive.setPoseEstimate(pose);
    }
    public void lineTo(double x, double y, double r){
        setXTarget(x);
        setYTarget(y);
        setRTarget(r);
    }
    public void lineToCHeading(double x, double y){
        setXTarget(x);
        setYTarget(y);
    }
    public void lineToChangeHeadingUnderCondition(double x, double y, double r, boolean condition){
        setXTarget(x);
        setYTarget(y);
        if(condition){
            setRTarget(r);
        }
    }
    public void setMaxPower(double maxPower){
        this.maxPower = maxPower;
    }
    public void setOn(boolean a){
        this.on = a;
    }

    public void finishPurePersuiting(Pose2d endPoint){
        purePersuiting = false;
        setXTarget(endPoint.getX());
        setYTarget(endPoint.getY());
        setRTarget(endPoint.getHeading());
    }

    //Takes in current posx, posy, posr, targetx, and targety
    //calculates the angle to follow that requires the least rotation
    public double toPoint(double x, double y, double r, double x2, double y2){
        double x3 = x2 - x;
        double y3 = y2 - y;

        double x4 = Math.cos(r);
        double y4 = Math.sin(r);

        double dotProduct = x3 * x4 + y3 * y4;

        double magnitude = Math.sqrt(x3 * x3 + y3 * y3);
        double cosine = Math.acos(dotProduct / magnitude);
        double angle = Math.atan2(y3, x3);
        double finalAngle=0;

        if(nearlyEqual(Math.cos(angle), Math.cos(r+cosine))&&nearlyEqual(Math.sin(angle), Math.sin(r+cosine))){
            finalAngle = r+cosine;
        }
        if(nearlyEqual(Math.cos(angle), Math.cos(r-cosine))&&nearlyEqual(Math.sin(angle), Math.sin(r-cosine))){
            finalAngle = r-cosine;
        }


        return finalAngle;
    }
    public boolean nearlyEqual(double a, double b) {
        return Math.abs(a - b) < 1e-9;
    }

    public void setForceStop(boolean b){
        forceStop =b;
    }
    public boolean isForceStopped(){
        return forceStop;
    }

    public void setPathEndHold(boolean a){
        this.ending = a;
    }

    public void setFollowRadius(double radius){
        this.followRadius = radius;
    }


}
