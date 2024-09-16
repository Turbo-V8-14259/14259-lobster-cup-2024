package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class intake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        waitForStart();
        while(opModeIsActive()){
            intakeMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        }

    }
}
