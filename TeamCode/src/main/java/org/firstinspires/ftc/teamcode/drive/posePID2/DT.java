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
public class DT{

    private boolean forceStop = false;
    private SampleMecanumDrive drive;
    private VoltageSensor vs;
    private double compensator;
    private DcMotorEx leftFront, rightRear, rightFront, leftRear;

    private double xTarget, yTarget, rTarget;
    private double xRn, yRn, rRn;
    private double deltaX, deltaY, deltaR;
    private boolean isAtTarget;


    private double errorX, errorY; //PEE JAY

    public double turnVelocity;

    private boolean purePersuiting = false;

    private BasicPID xController, yController, rController, pprController, NxController, NyController;
    private PIDCoefficients xyCoeff, rCoeff, pprCoeff, NxyCoeff;

    private double xOut, yOut, rOut;
    private double twistedR, count, lastAngle;
    private double xPower, yPower;

    private boolean isAtXYTarget;
    private boolean ending = false;

    private double maxPower = 1;
    private double followRadius = 0;
    private double flPower, frPower, blPower, brPower;
    
    private double normalize;
    private boolean on = true;
    ElapsedTime timer = new ElapsedTime();
    double currentTime = 0;
    double lastTime = 0;
    double currentHeading = 0;
    double currentX = 0;
    double lastX = 0;
    double lastHeading = 0;
    double xVelocity=0;
    double deltaTime = 0;
    double lastY = 0;
    double currentY = 0;
    double yVelocity = 0;
    public DT(HardwareMap hardwareMap, ElapsedTime timer){
        this.timer = timer;
        this.vs = hardwareMap.voltageSensor.iterator().next();
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.drive.setPoseEstimate(new Pose2d(0, 0, 0));
        this.leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "br");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        this.rightRear.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.leftRear.setDirection(DcMotor.Direction.REVERSE);
        this.xyCoeff = new PIDCoefficients(DTConstants.xyP, DTConstants.xyI, DTConstants.xyD);
        this.NxyCoeff = new PIDCoefficients(DTConstants.NxyP, DTConstants.NxyI, DTConstants.NxyD);
        this.rCoeff = new PIDCoefficients(DTConstants.rP, DTConstants.rI, DTConstants.rD);
        this.pprCoeff = new PIDCoefficients(DTConstants.pPrP, DTConstants.pPrI, DTConstants.pPrD);
        this.xController = new BasicPID(xyCoeff);
        this.yController = new BasicPID(xyCoeff);
        this.rController = new BasicPID(rCoeff);
        this.pprController = new BasicPID(pprCoeff);
        this.NxController = new BasicPID(NxyCoeff);
        this.NyController = new BasicPID(NxyCoeff);

        this.xTarget = 0;
        this.yTarget = 0;
        this.rTarget = 0;
        if(drive.getPoseEstimate().getX() != 0 || drive.getPoseEstimate().getY() != 0 || drive.getPoseEstimate().getHeading() != 0){
            drive.setPoseEstimate(new Pose2d(0, 0, 0));
        }
    }

    public DT(HardwareMap hardwareMap){
        this.vs = hardwareMap.voltageSensor.iterator().next();
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.drive.setPoseEstimate(new Pose2d(0, 0, 0));
        this.leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "br");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        this.rightRear.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.leftRear.setDirection(DcMotor.Direction.REVERSE);
        this.xyCoeff = new PIDCoefficients(DTConstants.xyP, DTConstants.xyI, DTConstants.xyD);
        this.rCoeff = new PIDCoefficients(DTConstants.rP, DTConstants.rI, DTConstants.rD);
        this.pprCoeff = new PIDCoefficients(DTConstants.pPrP, DTConstants.pPrI, DTConstants.pPrD);
        this.xController = new BasicPID(xyCoeff);
        this.yController = new BasicPID(xyCoeff);
        this.rController = new BasicPID(rCoeff);
        this.NxyCoeff = new PIDCoefficients(DTConstants.NxyP, DTConstants.NxyI, DTConstants.NxyD);
        this.pprController = new BasicPID(pprCoeff);
        this.NxController = new BasicPID(NxyCoeff);
        this.NyController = new BasicPID(NxyCoeff);
        this.xTarget = 0;
        this.yTarget = 0;
        this.rTarget = 0;
        if(drive.getPoseEstimate().getX() != 0 || drive.getPoseEstimate().getY() != 0 || drive.getPoseEstimate().getHeading() != 0){
            drive.setPoseEstimate(new Pose2d(0, 0, 0));
        }
    }
    public DT(HardwareMap hardwareMap, Pose2d startPose){
        this.vs = hardwareMap.voltageSensor.iterator().next();
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.drive.setPoseEstimate(startPose);
        this.leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "br");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        this.rightRear.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.leftRear.setDirection(DcMotor.Direction.REVERSE);
        this.xyCoeff = new PIDCoefficients(DTConstants.xyP, DTConstants.xyI, DTConstants.xyD);
        this.rCoeff = new PIDCoefficients(DTConstants.rP, DTConstants.rI, DTConstants.rD);
        this.pprCoeff = new PIDCoefficients(DTConstants.pPrP, DTConstants.pPrI, DTConstants.pPrD);
        this.xController = new BasicPID(xyCoeff);
        this.yController = new BasicPID(xyCoeff);
        this.rController = new BasicPID(rCoeff);
        this.pprController = new BasicPID(pprCoeff);

        this.xTarget = startPose.getX();
        this.yTarget = startPose.getY();
        this.rTarget = startPose.getHeading();
    }
    public DT(HardwareMap hardwareMap, Pose2d startPose, boolean isPurePersuiting){
        this.vs = hardwareMap.voltageSensor.iterator().next();
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.drive.setPoseEstimate(startPose);
        this.leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "br");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        this.rightRear.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.leftRear.setDirection(DcMotor.Direction.REVERSE);
        this.xyCoeff = new PIDCoefficients(DTConstants.xyP, DTConstants.xyI, DTConstants.xyD);
        this.rCoeff = new PIDCoefficients(DTConstants.rP, DTConstants.rI, DTConstants.rD);
        this.pprCoeff = new PIDCoefficients(DTConstants.pPrP, DTConstants.pPrI, DTConstants.pPrD);
        this.xController = new BasicPID(xyCoeff);
        this.yController = new BasicPID(xyCoeff);
        this.rController = new BasicPID(rCoeff);
        this.pprController = new BasicPID(pprCoeff);

        this.xTarget = startPose.getX();
        this.yTarget = startPose.getY();
        this.rTarget = startPose.getHeading();

        this.setPurePersuiting(isPurePersuiting);
    }
    public void setPowers(double y, double x, double r){
        if(!forceStop){
            normalize = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
            flPower = (y+x+r);
            blPower = (y-x+r);
            brPower = (y+x-r);
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
            if (!ending) {
                lastHeading = currentHeading;

                drive.updatePoseEstimate();
                xRn = drive.getPoseEstimate().getX();
                yRn = drive.getPoseEstimate().getY();
                rRn = drive.getPoseEstimate().getHeading();
                xOut = xController.calculate(xTarget, xRn);
                yOut = -yController.calculate(yTarget, yRn);
//        double n = Math.hypot(xOut,yOut);
//        xOut = maxPower * Math.signum(xOut);
//        yOut = maxPower * Math.signum(yOut);
            xOut *= maxPower / followRadius;
            yOut *= maxPower / followRadius;

                deltaY = yTarget - yRn;
                deltaX = xTarget - xRn;
//        xOut = xController.calculate(xTarget, xRn);
//        yOut = -yController.calculate(yTarget, yRn);

                if (Math.abs(rRn - lastAngle) > M.PI) count += Math.signum(lastAngle - rRn);
                lastAngle = rRn;
                twistedR = count * (2 * M.PI) + rRn;
                currentHeading = twistedR;
                lastTime = currentTime;
                currentTime = timer.nanoseconds() / 1000000;
                deltaTime = currentTime - lastTime;

                turnVelocity = (currentHeading - lastHeading) / (deltaTime);
                xVelocity = (currentX - lastX) / (deltaTime);
                yVelocity = (currentY - lastY) / (deltaTime);
                rOut = -((rTarget - twistedR) * 1.3 - turnVelocity * 105);
//        rOut = -rController.calculate(rTarget, twistedR);
                xPower = (xOut * T.cos(rRn) - yOut * T.sin(rRn));
                yPower = (xOut * T.sin(rRn) + yOut * T.cos(rRn));

                deltaR = rTarget - twistedR;

                if (Math.abs(xPower) > DTConstants.maxAxialPower)
                    xPower = DTConstants.maxAxialPower * Math.signum(xPower);
                if (Math.abs(yPower) > DTConstants.maxAxialPower)
                    yPower = DTConstants.maxAxialPower * Math.signum(yPower);
                if (Math.abs(rOut) > DTConstants.maxAngularPower)
                    rOut = DTConstants.maxAngularPower * Math.signum(rOut);
//
//        if(Math.abs(xPower) < 0.05) xPower = 0;
//        else xPower += DTConstants.XYBasePower * Math.signum(xPower)* 1/maxPower;
//        if(Math.abs(yPower) < 0.05) yPower = 0;
//        else yPower += DTConstants.XYBasePower * Math.signum(yPower) * 1/maxPower;
//        if (Math.abs(deltaR) < DTConstants.allowedAngularError) rOut = 0;
//        else rOut += DTConstants.RBasePower * Math.signum(rOut) * 1/maxPower;

                double zeroMoveAngle = Math.toRadians(25);
                double errorScale = 1 - (Math.abs(deltaR) / zeroMoveAngle);
                if (errorScale < 0) {
                    errorScale = 0;
                }

                xPower *= errorScale;
                yPower *= errorScale;

//            compensator = vs.getVoltage() / 12.5;
//            xPower/=compensator;
//            yPower/=compensator;
//            rOut/=compensator;

//        setPowers(xPower, yPower,rOut);
//        setPowers(-xPower, 0,0);
                setPowers(-xPower, -yPower, rOut);
//        setPowers(0, 0,rOut);


                if (Math.abs(deltaX) < 2 && Math.abs(deltaY) < 2 && Math.abs(deltaR) < DTConstants.allowedAngularError) {
                    isAtTarget = true;
                } else {
                    isAtTarget = false;
                }

            } else {
                lastHeading = currentHeading;
                drive.updatePoseEstimate();
                xRn = drive.getPoseEstimate().getX();
                currentX = xRn;
                yRn = drive.getPoseEstimate().getY();
                currentY = yRn;
                rRn = drive.getPoseEstimate().getHeading();

//            xOut = NxController.calculate(xTarget, xRn);
//            yOut = -NyController.calculate(yTarget, yRn);
//        double n = Math.hypot(xOut,yOut);
//        xOut = maxPower * Math.signum(xOut);
//        yOut = maxPower * Math.signum(yOut);

//            deltaY = yTarget - yRn;
//            deltaX = xTarget - xRn;
//        xOut = xController.calculate(xTarget, xRn);
//        yOut = -yController.calculate(yTarget, yRn);

                if (Math.abs(rRn - lastAngle) > M.PI) count += Math.signum(lastAngle - rRn);
                lastAngle = rRn;
                twistedR = count * (2 * M.PI) + rRn;


                currentHeading = twistedR;

                lastTime = currentTime;

                currentTime = timer.nanoseconds() / 1000000;


                deltaTime = currentTime - lastTime;

                turnVelocity = (currentHeading - lastHeading) / (deltaTime);
                xVelocity = (currentX - lastX) / (deltaTime);
                yVelocity = (currentY - lastY) / (deltaTime);

                xOut = ((xTarget - currentX) * .075 - xVelocity * 0);
                yOut = -((yTarget - currentY) * .075 - yVelocity * 0);
                rOut = -((rTarget - twistedR) * 1.3 - turnVelocity * 105);
//        rOut = -rController.calculate(rTarget, twistedR);
                xPower = (xOut * T.cos(rRn) - yOut * T.sin(rRn));
                yPower = (xOut * T.sin(rRn) + yOut * T.cos(rRn));

                deltaR = rTarget - twistedR;

//            if(Math.abs(xPower) > DTConstants.maxAxialPower) xPower = DTConstants.maxAxialPower * Math.signum(xPower);
//            if(Math.abs(yPower) > DTConstants.maxAxialPower) yPower = DTConstants.maxAxialPower * Math.signum(yPower);
//            if(Math.abs(rOut) > DTConstants.maxAngularPower) rOut = DTConstants.maxAngularPower * Math.signum(rOut);
//
//        if(Math.abs(xPower) < 0.05) xPower = 0;
//        else xPower += DTConstants.XYBasePower * Math.signum(xPower)* 1/maxPower;
//        if(Math.abs(yPower) < 0.05) yPower = 0;
//        else yPower += DTConstants.XYBasePower * Math.signum(yPower) * 1/maxPower;
//        if (Math.abs(deltaR) < DTConstants.allowedAngularError) rOut = 0;
//        else rOut += DTConstants.RBasePower * Math.signum(rOut) * 1/maxPower;

                double zeroMoveAngle = Math.toRadians(25);
                double errorScale = 1 - (Math.abs(deltaR) / zeroMoveAngle);
                if (errorScale < 0) {
                    errorScale = 0;
                }

                xPower *= errorScale;
                yPower *= errorScale;

//            compensator = vs.getVoltage() / 12.5;
//            xPower/=compensator;
//            yPower/=compensator;
//            rOut/=compensator;

//        setPowers(xPower, yPower,rOut);
//        setPowers(-xPower, 0,0);
                setPowers(-xPower, -yPower, rOut);
//        setPowers(0, 0,rOut);


                if (Math.abs(deltaX) < 2 && Math.abs(deltaY) < 2 && Math.abs(deltaR) < DTConstants.allowedAngularError) {
                    isAtTarget = true;
                } else {
                    isAtTarget = false;
                }
                lastX = currentX;
                lastY = currentY;
            }
    }

    public void setPurePersuiting(boolean isPurePersuiting){
        purePersuiting = isPurePersuiting;
    }
    public Pose2d getLocation(){
        return new Pose2d(xRn, yRn, rRn);
    }
    public void setXTarget(double x){
        this.xTarget = x;
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
