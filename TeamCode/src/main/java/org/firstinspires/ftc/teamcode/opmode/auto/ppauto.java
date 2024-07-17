package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.path.PurePursuitPath;
import org.firstinspires.ftc.teamcode.drive.path.PurePursuitUtil;
import org.firstinspires.ftc.teamcode.drive.posePID2.DT;

@Autonomous
@Config
public class ppauto extends LinearOpMode {
    DT drive = new DT(hardwareMap);
    PurePursuitPath path = new PurePursuitPath(drive, 13,13,new Pose2d(0, 0),new Pose2d(0, 60),new Pose2d(20, 0));

    double movePower =1;
    double headingOffset=0;
    @Override
    public void runOpMode() throws InterruptedException {
        path.init(movePower, headingOffset);
        waitForStart();

        while(opModeIsActive()){
            path.update();
        }

    }
}
