package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "pp comparison")
@Config
public class xyrLetsGo extends LinearOpMode {
    Pose2d target1 = new Pose2d(0, 0);
    Pose2d target2 = new Pose2d(10,30);
    Pose2d target3 = new Pose2d(57,30);
    Pose2d target4=new Pose2d(57,5);
    Pose2d target5 = new Pose2d(1,1);

    double state = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            if(state == 0){
                double newAngle = drive.toPoint(drive.getX(), drive.getY(), drive.getR(), target1.getX(), target1.getY());
                drive.lineTo(target1.getX(),target1.getY(),newAngle);
                if(Math.abs(Math.hypot((target1.getX()-drive.getX()), (target1.getY() - drive.getY()))) < 3){
                    state++;
                }
            }else if(state == 1){
                double newAngle = drive.toPoint(drive.getX(), drive.getY(), drive.getR(), target2.getX(), target2.getY());
                drive.lineTo(target2.getX(),target2.getY(),newAngle);
                if(Math.abs(Math.hypot((target2.getX()-drive.getX()), (target2.getY() - drive.getY()))) < 3){
                    state++;
                }
            }else if(state == 2){
                double newAngle = drive.toPoint(drive.getX(), drive.getY(), drive.getR(), target3.getX(), target3.getY());
                drive.lineTo(target3.getX(),target3.getY(),newAngle);
                if(Math.abs(Math.hypot((target3.getX()-drive.getX()), (target3.getY() - drive.getY()))) < 3){
                    state++;
                }
            }else if(state == 3){
                double newAngle = drive.toPoint(drive.getX(), drive.getY(), drive.getR(), target4.getX(), target4.getY());
                drive.lineTo(target4.getX(),target4.getY(),newAngle);
                if(Math.abs(Math.hypot((target4.getX()-drive.getX()), (target4.getY() - drive.getY()))) < 3){
                    state++;
                }
            }else if(state == 4){
                double newAngle = drive.toPoint(drive.getX(), drive.getY(), drive.getR(), target5.getX(), target5.getY());
                drive.lineTo(target5.getX(),target5.getY(),newAngle);
                if(Math.abs(Math.hypot((target5.getX()-drive.getX()), (target5.getY() - drive.getY()))) < 3){
                    state++;
                }
            }else if(state == 5){

            }
            drive.update();
            telemetry.update();
        }
    }
}
