///*
//package org.firstinspires.ftc.teamcode.drive.path;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import java.util.ArrayList;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
//import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
//
//@TeleOp (name = "Debug PP")
//@Config
//public class pathtest extends LinearOpMode {
//    ElapsedTime timer = new ElapsedTime();
//    double previousTime = 0;
//    double currentTime = 0;
//
//    double lookaheadRadius = 20;
//
//    DT drive;
//
//    int i = 0;
//    double size = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        stickyGamepad gamepadOne;
//
//
//        drive = new DT(hardwareMap);
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        drive.setFollowRadius(lookaheadRadius);
//        PurePursuitUtil.updateSegment(1);
//        PurePursuitUtil.updateEnding(false);
//        drive.setOn(true);
//        gamepadOne = new stickyGamepad(gamepad1);
//        waitForStart();
//        while(opModeIsActive()) {
////            drive.setPathEndHold(true);
//            previousTime = currentTime;
//            currentTime = timer.nanoseconds()/1000000000.0;
//
//
//            if(gamepadOne.b){
////                wayPoints.add(new Pose2d(drive.getX(), drive.getY()));
//                size++;
//            }
//            ArrayList<Pose2d> wayPoints = new ArrayList<>();
//            wayPoints.add(new Pose2d(0, 0));
//            wayPoints.add(new Pose2d(0, 50));
//            wayPoints.add(new Pose2d(50, 50));
//            wayPoints.add(new Pose2d(50, 0));
//            wayPoints.add(new Pose2d(0, 0));
//            followPath(wayPoints, 1, 30, 30, 0, -99, drive);
////            wayPoints.add(new Pose2d(50, 60));
////
////            wayPoints.add(new Pose2d(70, 30));
//
////            if(gamepad1.dpad_down){
////                followPath(wayPoints, 1, 13, 13, 0, -99, drive);
////                drive.setOn(true);
////            }else{
////                drive.setPowers(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
////            }
//
//
//            telemetry.addData("i ", i);
//            telemetry.addData("hz ", 1/(currentTime-previousTime));
//            telemetry.addData("pose x", drive.getX());
//            telemetry.addData("pose y", drive.getY());
//            telemetry.addData("pose x", drive.getR());
//            telemetry.addData("size", size);
//
//
//            telemetry.addData("segment ", PurePursuitUtil.getSegment());
//            telemetry.update();
//
//            drive.update();
//            gamepadOne.update();
//        }
//
//
//    }
//
//    Pose2d lastTranslatePoint = new Pose2d(0,0);
//    Pose2d lastHeadingPoint = new Pose2d(0,0);
//    public void followPath(ArrayList<Pose2d> path, double movePower, double headingRadius, double moveRadius, double headingOffset, double lockAngle, DT drive) {
//        //false for urm non heading ig
//        //getFuturePos for vel extrapolation
//        Pose2d followDrive = PurePursuitUtil.followMe(path, drive.getFuturePos(500), moveRadius, lastTranslatePoint, false);
//        lastTranslatePoint = followDrive;
//
//        //true for heading
//
//        Pose2d followHeading = PurePursuitUtil.followMe(path, drive.getFuturePos(500), headingRadius, lastHeadingPoint, true);
//        lastHeadingPoint = followHeading;
//        if(PurePursuitUtil.getEnding()) {
//            drive.setPathEndHold(true);
//            drive.lineTo(followDrive.getX(), followDrive.getY(),Math.toRadians(-180));
//        }else{
//            drive.lineTo(followDrive.getX(), followDrive.getY(),drive.toPoint(drive.getX(), drive.getY(), drive.getR(), followHeading.getX(), followHeading.getY() + headingOffset));
//        }
//
//
//        drive.setMaxPower(movePower);
//
//
//
//    }
//
//
//}*/
