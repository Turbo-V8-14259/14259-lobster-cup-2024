
package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.path.PurePursuitPath;
import org.firstinspires.ftc.teamcode.drive.path.PurePursuitUtil;
import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
import org.firstinspires.ftc.teamcode.drive.posePID2.NewDT;
import org.firstinspires.ftc.teamcode.hardware.ClawWrist;
import org.firstinspires.ftc.teamcode.hardware.SATest;
import org.firstinspires.ftc.teamcode.vision.CameraPipelineBlue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
@Config
public class ppautotest extends LinearOpMode {
    enum State{
        FIRST,
        SECOND,
        THIRD
    }

    NewDT drive;
    PurePursuitPath path;
    ElapsedTime timer = new ElapsedTime();
    double movePower =1;
    double headingOffset=0;
    public static double moveRadius = 20;
    public static double headingRadius = 20;
    State currentState = State.FIRST;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NewDT(hardwareMap, new Pose2d(0, 0), timer);
        path = new PurePursuitPath(drive,moveRadius, headingRadius,
        new Pose2d(0, 0),
        new Pose2d(0, 50),
        new Pose2d(50, 50),
        new Pose2d(50, 0),
        new Pose2d(0, 0)

                );

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        path.init(movePower, headingOffset);
        waitForStart();

        while(opModeIsActive()){
            path.update();
            telemetry.addData("pose x", drive.getX());
            telemetry.addData("pose y", drive.getY());
            telemetry.addData("pose r", drive.getR());
            telemetry.addData("nanoseconds", drive.getTimer());


            telemetry.addData("segment ", PurePursuitUtil.getSegment());
            telemetry.update();

        }
    }
}
