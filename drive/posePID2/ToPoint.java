package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@TeleOp (name = "toPoint")
@Config
public class ToPoint extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            drive.setXTarget(drive.getX());
            drive.setYTarget(drive.getY());
            double newAngle = drive.toPoint(drive.getX(), drive.getY(), drive.getR(), 0,10);
            drive.setRTarget(newAngle);
            drive.update();
            telemetry.addData("Current robot postion X:", drive.getX());
            telemetry.addData("Current robot postion Y:", drive.getY());
            telemetry.addData("Current robot postion R:", Math.toDegrees(drive.getR()));
            telemetry.addData("new angle:", Math.toDegrees(newAngle));
            telemetry.update();
        }
    }
}
