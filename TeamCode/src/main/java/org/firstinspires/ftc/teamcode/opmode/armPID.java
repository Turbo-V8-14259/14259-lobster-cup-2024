package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
import org.firstinspires.ftc.teamcode.hardware.SATest;
import org.firstinspires.ftc.teamcode.hardware.SlidesArm;
@TeleOp
@Config

public class armPID extends LinearOpMode {
    public static double degreeTarget = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SATest slidesArm = new SATest(hardwareMap, timer);
        slidesArm.stopAndResetEncoder();

        waitForStart();
        while (opModeIsActive()){
            drive.setPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            slidesArm.setDegrees(degreeTarget);
            slidesArm.update();
            telemetry.addData("angle", slidesArm.getCurrentDegrees());
            telemetry.addData("error", slidesArm.getError());
            telemetry.addData("arm speed", slidesArm.getArmSpeed());
            telemetry.update();
        }
    }
}
