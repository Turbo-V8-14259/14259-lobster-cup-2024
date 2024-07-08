package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
@Disabled

public class onlyX extends LinearOpMode {
    public static double x = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            drive.setXTarget(x);
            drive.update();
            telemetry.addData("x pos", drive.getX());
            telemetry.addData("x power", drive.getPowerX());
            telemetry.update();
        }
    }
}
