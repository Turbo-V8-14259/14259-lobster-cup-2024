//package org.firstinspires.ftc.teamcode.opmode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
//
//
//@TeleOp
//@Disabled
//
//public class driveTest extends LinearOpMode {
//    DT drive;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        drive = new DT(hardwareMap);
//        waitForStart();
//        while(opModeIsActive()){
//            drive.setPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
//        }
//    }
//}
