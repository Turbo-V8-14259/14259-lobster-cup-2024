package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
import org.firstinspires.ftc.teamcode.hardware.ClawWrist;
import org.firstinspires.ftc.teamcode.hardware.SlidesArm;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;

@TeleOp(name = "probably tele")
@Config
public class clawWristTest extends LinearOpMode {
    int intakeState = 0;
    boolean timeToggle = true;
    double TimeStamp = 0;
    ElapsedTime timer = new ElapsedTime();
    public static double target = 0;
    DcMotorEx armMotor;
    DcMotorEx br;

    String bigState = "INTERMEDIATE";
    int intakeExtension = 10;
    int depositExtension = 12;
    int armAngle = 130;
    DT drive;
    int depoFSM = 0;

    int clawL = 0;
    int clawR = 0;
    boolean clawOverride = false;
    boolean leftToggle = false;
    boolean rightToggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DT(hardwareMap);
        CRServo one = hardwareMap.get(CRServo.class, "hl"); //works port1
        CRServo two = hardwareMap.get(CRServo.class, "hr");
        one.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        SlidesArm slidesArm = new SlidesArm(hardwareMap);

        stickyGamepad gamepad = new stickyGamepad(gamepad1);
        stickyGamepad gamepadTwo = new stickyGamepad(gamepad2);

//        armMotor = hardwareMap.get(DcMotorEx.class, "a");
//        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        br = hardwareMap.get(DcMotorEx.class, "br");
//        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            drive.setPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            if(bigState.equals("INTERMEDIATE")){
                slidesArm.setDegrees(20);
                clawWrist.setWristState(ClawWrist.WristState.NEUTRAL);
                if(!clawOverride){
                    clawWrist.setClawState(ClawWrist.ClawState.CLOSED);
                }
                slidesArm.setInches(1);
            }else if(bigState.equals("INTAKE1")){
                clawOverride = false;
                slidesArm.setDegrees(15);
                clawWrist.setWristState(ClawWrist.WristState.INTAKE);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds() > TimeStamp + 500){
                    timeToggle = true;
                    bigState = "INTAKEREADY";
                }
            }else if(bigState.equals("INTAKEREADY")){
                clawWrist.setClawState(ClawWrist.ClawState.OPEN);
                slidesArm.setDegrees(3);
                slidesArm.setInches(intakeExtension);
            }else if(bigState.equals("INTAKECLOSECLAW")) {
                clawWrist.setClawState(ClawWrist.ClawState.CLOSED);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if (timer.milliseconds() > TimeStamp + 500) {
                    timeToggle = true;
                    bigState = "INTERMEDIATE";
                }
            }else if(bigState.equals("ARMFLIP")){
                slidesArm.setDegrees(armAngle);
                clawWrist.setWristState(ClawWrist.WristState.OUTTAKE);
                clawWrist.alignBoard(-slidesArm.getCurrentDegrees());
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if (timer.milliseconds() > TimeStamp + 1000) {
                    timeToggle = true;
                    bigState = "SLIDESOUT";
                }
            }else if(bigState.equals("SLIDESOUT")){
                slidesArm.setInches(depositExtension);
                slidesArm.setDegrees(armAngle);
                clawWrist.setWristState(ClawWrist.WristState.OUTTAKE);
                clawWrist.alignBoard(-slidesArm.getCurrentDegrees());
            }else if(bigState.equals("SCORE")){
                if(!clawOverride){
                    clawWrist.setClawState(ClawWrist.ClawState.OPEN);
                }
            }



//
//            if(gamepad.left_bumper){
//                bigState = "INTAKE1";
//            }else if(gamepad.right_bumper){
//                bigState = "INTAKECLOSECLAW";
//            }

            if(gamepadTwo.right_bumper){
                depoFSM++;
                if(depoFSM==1){
                    bigState = "INTAKE1";
                }else if(depoFSM == 2){
                    bigState = "INTAKECLOSECLAW";
                }else if(depoFSM == 3) {
                    bigState = "ARMFLIP";
                }else if(depoFSM == 4){
                    bigState = "SCORE";
                }else if(depoFSM == 5){
                    bigState = "INTERMEDIATE";
                    depoFSM = 0;
                }
            }
            if(gamepadTwo.left_stick_button){
                depoFSM = 1;
                bigState = "INTAKE1";
            }
            if(gamepadTwo.dpad_left){
                clawOverride = true;
                leftToggle = !leftToggle;
                if(leftToggle){
                    clawWrist.setClawState(ClawWrist.ClawState.OPENLeft);
                }else if(!leftToggle){
                    clawWrist.setClawState(ClawWrist.ClawState.CLOSEDLeft);
                }
            }else if(gamepadTwo.dpad_right){
                clawOverride = true;
                rightToggle = !rightToggle;
                if(rightToggle){
                    clawWrist.setClawState(ClawWrist.ClawState.OPENRight);
                }else if(!rightToggle){
                    clawWrist.setClawState(ClawWrist.ClawState.CLOSEDRight);
                }
            }
//            if(gamepadTwo.left_bumper && !force){
//                forceLClawOpen = true;
//            }
//            if(gamepadTwo.b){
//                clawL++;
//            }
//            if(clawL > 2){
//                clawL = 0;
//            }
            one.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            two.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

            if(gamepadTwo.dpad_up){
                depositExtension+=5;
            }else if(gamepadTwo.dpad_down){
                depositExtension-=5;
            }else if(gamepadTwo.a)
            {
                armAngle+=10;
            }else if(gamepadTwo.y){
                armAngle-=10;
            }


            telemetry.addData("big state, ", bigState);
            telemetry.addData("depo fsm", depoFSM);
            telemetry.addData("arm in degrees", slidesArm.getCurrentDegrees());
            telemetry.update();
            gamepad.update();
            clawWrist.update();
            slidesArm.update();
            gamepadTwo.update();
        }
    }
}
