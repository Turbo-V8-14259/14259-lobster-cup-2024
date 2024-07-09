package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;

@TeleOp
public class wristBounds extends LinearOpMode {
    //after intake(into the robot): 0.25
    //max angle of claw: 205
    //claw neutral, resting on ground: 1
    double target = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "w");
        stickyGamepad gamepad = new stickyGamepad(gamepad1);
        servo.setPosition(0.5);

        waitForStart();
        while(opModeIsActive()){
            servo.setPosition(target);

            if(gamepad.left_bumper){
                target+=0.05;
            }else if(gamepad.right_bumper){
                target-=0.05;
            }
            telemetry.addData("one", target);
            telemetry.update();
            gamepad.update();
        }
    }
}
