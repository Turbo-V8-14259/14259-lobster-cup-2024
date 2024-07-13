package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
@Disabled

public class dtmotorTests extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        rightRear = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");

//        rightRear.setDirection(DcMotor.Direction.FORWARD);
//        rightFront.setDirection(DcMotor.Direction.FORWARD);
//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD); //turning reversed
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_left){
                leftFront.setPower(0.5);
            }else if(gamepad1.dpad_down){
                leftRear.setPower(0.5);
            }else if(gamepad1.dpad_right){
                rightFront.setPower(0.5);
            }else if(gamepad1.dpad_up){
                rightRear.setPower(0.5);
            }
        }
    }
}
