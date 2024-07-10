package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ClawWrist;
import org.firstinspires.ftc.teamcode.hardware.SlidesArm;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;

@TeleOp
@Config
public class clawWristTest extends LinearOpMode {
    int intakeState = 0;
    boolean timeToggle = true;
    double TimeStamp = 0;
    ElapsedTime timer = new ElapsedTime();
    public static double target = 0;
    DcMotorEx armMotor;
    DcMotorEx br;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        SlidesArm slidesArm = new SlidesArm(hardwareMap);
        slidesArm.stopAndResetEncoder();
        stickyGamepad gamepad = new stickyGamepad(gamepad1);
        armMotor = hardwareMap.get(DcMotorEx.class, "a");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        br = hardwareMap.get(DcMotorEx.class, "br");
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while(opModeIsActive()){
//            if(intakeState == 1){
//                clawWrist.setClawState(ClawWrist.ClawState.CLOSED);
//                if(timeToggle){//timeToggle starts at true by default
//                    TimeStamp = timer.milliseconds();
//                    timeToggle = false;
//                }
//                if(timer.milliseconds() > TimeStamp + 500){
//                    intakeState=2;
//                    timeToggle = true;
//                }
//            } else if(intakeState == 2){
//                slidesArm.setInches(target);
//                clawWrist.setWristState(ClawWrist.WristState.NEUTRAL);
//            } else if(intakeState == 0){
//                slidesArm.setInches(target);
//                clawWrist.setWristState(ClawWrist.WristState.INTAKE);
//
//                clawWrist.setClawState(ClawWrist.ClawState.OPEN);
//            }
//            if(gamepad.left_bumper){
//                intakeState = 0;
//            }else if(gamepad.right_bumper){
//                intakeState = 1; }
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
                slidesArm.setInches(target);
                //arm logic if fixed?
                clawWrist.setWristState(ClawWrist.WristState.OUTTAKE);
                double angle = br.getCurrentPosition() * .225;
                clawWrist.alignBoard(angle);
                clawWrist.update();
                if(timeToggle){//time toggle here to give time for it to align
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 500){
                    intakeState=3;
                    timeToggle = true;
                }
            } else if(intakeState == 3){
                clawWrist.setClawState(ClawWrist.ClawState.OPEN);
            } else if(intakeState == 0){
                slidesArm.setInches(target);
                clawWrist.setWristState(ClawWrist.WristState.INTAKE);

                clawWrist.setClawState(ClawWrist.ClawState.OPEN);
            }
            if(gamepad.left_bumper) {//intake pixels
                intakeState =0;
            }else if(gamepad.right_bumper) {//release pizels
                intakeState =1;
            }

            telemetry.addData("current inches ",  slidesArm.getCurrentInches());
            telemetry.addData("target inches", target);
            telemetry.update();
            gamepad.update();
            clawWrist.update();
            slidesArm.update();
        }
    }
}
