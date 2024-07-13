package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SlidesArm;

@TeleOp
public class SlidesReser extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SlidesArm slidesArm = new SlidesArm(hardwareMap);
        slidesArm.stopAndResetEncoder();

        waitForStart();

        while(opModeIsActive()){

        }
    }
}
