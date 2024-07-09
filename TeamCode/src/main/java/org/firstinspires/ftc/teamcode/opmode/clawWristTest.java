package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ClawWrist;
import org.firstinspires.ftc.teamcode.hardware.SlidesArm;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;

@TeleOp
public class clawWristTest extends LinearOpMode {
    int intakeState = 0;
    boolean timeToggle = true;
    double TimeStamp = 0;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        SlidesArm slidesArm = new SlidesArm(hardwareMap);
        stickyGamepad gamepad = new stickyGamepad(gamepad1);
        waitForStart();
        while(opModeIsActive()){
            if(intakeState == 1){
                clawWrist.setClawState(ClawWrist.ClawState.CLOSED);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 500){
                    intakeState=2;
                    timeToggle = true;
                }
            } else if(intakeState == 2){
                slidesArm.setInches(0);
                clawWrist.setWristState(ClawWrist.WristState.NEUTRAL);
            } else if(intakeState == 0){
                slidesArm.setInches(10);
                clawWrist.setWristState(ClawWrist.WristState.INTAKE);
                clawWrist.setClawState(ClawWrist.ClawState.OPEN);
            }
            if(gamepad.left_bumper){
                intakeState = 0;
            }else if(gamepad.right_bumper){
                intakeState = 1;
            }
            telemetry.addData("current inches ",  slidesArm.getCurrentInches());
            telemetry.update();
            gamepad.update();
            clawWrist.update();
            slidesArm.update();
        }
    }
}
