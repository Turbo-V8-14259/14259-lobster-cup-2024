package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
public class slidesBounds extends LinearOpMode {
    DcMotorEx slidesMotor;
    DcMotorEx bl;//80.25 inches is -1100 ticks, when i spin motor forward, tick value becomes negative.
    @Override
    public void runOpMode() throws InterruptedException {
        slidesMotor = hardwareMap.get(DcMotorEx.class, "s");
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl = hardwareMap.get(DcMotorEx.class, "bl"); //home is 0, 1120 is max on 0 degrees, 1190 on the max rotated point
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            slidesMotor.setPower(gamepad1.left_stick_y);
            telemetry.addData("slides encoder ", bl.getCurrentPosition());
            telemetry.update();
        }
    }
}
