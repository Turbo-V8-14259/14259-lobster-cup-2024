package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.ClawWrist;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;

@TeleOp
@Disabled

public class wristBounds extends LinearOpMode {
    //after intake(into the robot): 0.25
    //max angle of claw: 205
    //claw neutral, resting on ground: 1
    double target = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        stickyGamepad gamepad = new stickyGamepad(gamepad1);
        ClawWrist wrist = new ClawWrist(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            wrist.setWristState(ClawWrist.WristState.OUTTAKE);
            wrist.setAngle(90);
            wrist.update();
            telemetry.addData("target w", wrist.getTargetW());
            telemetry.update();
        }
    }
}
