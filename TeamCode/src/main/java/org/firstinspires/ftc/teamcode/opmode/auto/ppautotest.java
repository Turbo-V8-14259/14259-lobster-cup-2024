
package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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


@TeleOp
@Config
public class ppautotest extends LinearOpMode {
    enum State{
        FIRST,
        SECOND,
        THIRD
    }

    NewDT drive;
    PurePursuitPath path;
    ElapsedTime timer2 = new ElapsedTime();
    double previousTime = 0;
    double currentTime = 0;
    ElapsedTime timer = new ElapsedTime();
    public static double movePower =0.5;
    double headingOffset=0;
    public static double moveRadius = 10;
    public static double headingRadius = 13;
    public static double dist=30;
    State currentState = State.FIRST;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NewDT(hardwareMap, new Pose2d(0, 0), timer);
        path = new PurePursuitPath(drive,moveRadius, headingRadius,
        new Pose2d(0, 0),
        new Pose2d(50, 0),
        new Pose2d(50, 50),
        new Pose2d(0, 50),
        new Pose2d(0, 0)
                );

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        path.init(movePower, headingOffset);
        waitForStart();

        while(opModeIsActive()){
            previousTime = currentTime;
            currentTime = timer2.nanoseconds()/1000000000.0;

            PurePursuitUtil.setPathLength(path.getPathLength());
            path.update();
            telemetry.addData("hz ", 1/(currentTime-previousTime));
            telemetry.addData("pose x", drive.getX());
            telemetry.addData("pose y", drive.getY());
            telemetry.addData("pose r", drive.getR());

            telemetry.addData("drive x target", path.getFollowDrive().getX());
            telemetry.addData("drive y target", path.getFollowDrive().getY());

            telemetry.addData("heading x target", path.getFollowHeading().getX());
            telemetry.addData("heading y target", path.getFollowHeading().getY());

            telemetry.addData("path length", path.getPathLength());
            telemetry.addData("move segment ", PurePursuitUtil.getMoveSegment());
            telemetry.addData("heading segment ", PurePursuitUtil.getHeadingSegment());
            telemetry.update();

        }
    }
}
