package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.ClawWrist;

@TeleOp
public class armBounds extends LinearOpMode {
    DcMotorEx armMotor;
    DcMotorEx br;
    @Override
    public void runOpMode() throws InterruptedException {
        //800 is 180,
        // 0 is 0
        armMotor = hardwareMap.get(DcMotorEx.class, "a");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ClawWrist wrist = new ClawWrist(hardwareMap);

        br = hardwareMap.get(DcMotorEx.class, "br");
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wrist.setWristState(ClawWrist.WristState.OUTTAKE);

        waitForStart();
        while(opModeIsActive()){
            double angle = br.getCurrentPosition() * .225;
            wrist.alignBoard(angle);
            wrist.update();
//            armMotor.setPower(-0.2 * Math.cos(Math.toRadians(angle)));
            telemetry.addData("motor power", armMotor.getPower());
            telemetry.addData("arm angle ", angle);
            telemetry.update();
        }
    }
}
