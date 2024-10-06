package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.path.PurePursuitUtil;

import java.util.ArrayList;

@TeleOp (name = "lineTo")
@Config
public class lineToTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    private ArrayList<Pose2d> wayPoints = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        NewDT drive = new NewDT(hardwareMap, new Pose2d(0,0,Math.toRadians(0)), timer);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        wayPoints.add(new Pose2d(0,20));
        wayPoints.add(new Pose2d(20,20));
        wayPoints.add(new Pose2d(20,0));
        wayPoints.add(new Pose2d(0,0));
//
        drive.setPPConstants(1,1,0.5);
        drive.newPath(wayPoints, 0, false);

        waitForStart();
        while(opModeIsActive()){


            telemetry.addData("x: ", drive.getX());
            telemetry.addData("y: ", drive.getY());
            telemetry.addData("angle: ", Math.toDegrees(drive.getR()));
            telemetry.addData("pathLenght", drive.getPathLength() );
            telemetry.addData("point x", PurePursuitUtil.getInt().getX());
            telemetry.addData("point x", PurePursuitUtil.getInt().getY());
            telemetry.addData("xout", drive.getRawX());
            telemetry.addData("yout", drive.getRawY());
            telemetry.addData("x power", drive.getPowerX());
            telemetry.addData("y power", drive.getPowerY());
            telemetry.update();
            drive.followDrive();

        }
    }
}
