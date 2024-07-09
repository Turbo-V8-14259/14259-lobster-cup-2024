package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
@TeleOp
public class hangServos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo one = hardwareMap.get(CRServo.class, "hl"); //works port1
        CRServo two = hardwareMap.get(CRServo.class, "hr");

        stickyGamepad gamepad = new stickyGamepad(gamepad1);
        one.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()){
            one.setPower(gamepad1.left_stick_y);
            two.setPower(gamepad1.left_stick_y);
        }
    }
}
