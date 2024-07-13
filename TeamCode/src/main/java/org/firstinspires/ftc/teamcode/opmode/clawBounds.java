package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
@TeleOp
@Disabled

public class clawBounds extends LinearOpMode {
    double targetOne = 0.5;
    double targetTwo = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo one = hardwareMap.get(Servo.class, "c1"); //left, open is 0.15, 0.53 is closed
        Servo two = hardwareMap.get(Servo.class, "c2"); //.87 is open, closed is 0.49
        stickyGamepad gamepad = new stickyGamepad(gamepad1);
        one.setPosition(0.5);
        two.setPosition(0.5);
        waitForStart();
        while (opModeIsActive()) {
            one.setPosition(targetOne);
            two.setPosition(targetTwo);
            if (gamepad.left_bumper) {
                targetOne+=0.01;
            } else if (gamepad.right_bumper) {
                targetOne-=0.01;
            }
            if (gamepad.a) {
                targetTwo+=0.01;
            } else if (gamepad.b) {
                targetTwo-=0.01;
            }
            telemetry.addData("one", targetOne);
            telemetry.addData("two", targetTwo);
            telemetry.update();
            gamepad.update();
        }
    }
}
