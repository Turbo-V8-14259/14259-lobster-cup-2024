package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config

public class onlyXY extends LinearOpMode {
    public static double y = 0.0, x = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            drive.setYTarget(y);
            drive.setXTarget(x);
            drive.setRTarget(0);
            drive.setMaxPower(.5);
            drive.update();
            telemetry.addData("y pos", drive.getY());
            telemetry.addData("y output raw", drive.getRawY());
            telemetry.addData("y power", drive.getPowerY());
            telemetry.addData("y target", drive.getYTarget());
            telemetry.addData("r", drive.getR());
            telemetry.addData("sin(r)", Math.sin(drive.getR()));
            telemetry.update();
        }
    }
}
