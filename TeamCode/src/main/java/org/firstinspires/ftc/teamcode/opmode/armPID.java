package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.hardware.SlidesArm;
@TeleOp
@Config
public class armPID extends LinearOpMode {
    public static double degreeTarget = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SlidesArm slidesArm = new SlidesArm(hardwareMap);
        slidesArm.stopAndResetEncoder();

        waitForStart();
        while (opModeIsActive()){
            slidesArm.setDegrees(degreeTarget);
            slidesArm.update();
            telemetry.addData("angle", slidesArm.getCurrentDegrees());
            telemetry.update();
        }
    }
}
