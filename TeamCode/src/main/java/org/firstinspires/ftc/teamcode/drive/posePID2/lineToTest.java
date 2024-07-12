package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@TeleOp (name = "lineTo")
@Config
public class lineToTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            drive.lineTo(0,0,Math.toRadians(90));
            drive.update();
            telemetry.addData("x: ", drive.getX());
            telemetry.addData("y: ", drive.getY());
            telemetry.addData("r: ", drive.getR());

            telemetry.update();
        }
    }
}
