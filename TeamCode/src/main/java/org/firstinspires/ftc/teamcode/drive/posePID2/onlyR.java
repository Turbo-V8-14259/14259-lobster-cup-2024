package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config
public class onlyR extends LinearOpMode {
    public static double r = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap, new Pose2d(0,0,0), new ElapsedTime());
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            drive.setRTarget(Math.toRadians(r));
            drive.update();
            telemetry.addData("rotational angle rn", Math.toDegrees(drive.getR()));
            telemetry.addData("twisted R", Math.toDegrees(drive.getTwistedR()));
            telemetry.addData("raw R output",drive.getPowerR());
            telemetry.addData("turn velocuty",drive.getTurnVeloctiy());
            telemetry.addData("current time",drive.getCurrentTime());
            telemetry.update();
        }
    }
}
