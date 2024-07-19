
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
import org.firstinspires.ftc.teamcode.drive.path.PurePursuitPath2;
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
public class ppauto extends LinearOpMode {

    enum State {
        FIRST,
        SECOND,
        THIRD
    }
    DT drive;
    PurePursuitPath2 path1;
    PurePursuitPath2 path2;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timerArm = new ElapsedTime();
    boolean timeToggle = true;
    double TimeStamp = 0;

    double movePower =1;
    double headingOffset=0;
    public static double moveRadius = 20;
    public static double headingRadius = 20;
    State currentState = State.FIRST;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DT(hardwareMap, new Pose2d(0, 0), timer);
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        SATest slidesArm = new SATest(hardwareMap, timerArm);

        path1 = new PurePursuitPath2(drive,moveRadius, headingRadius,
                new Pose2d(0, 0),
                new Pose2d(0, 50),
                new Pose2d(50, 50),
                new Pose2d(50, 0),
                new Pose2d(0, 0)
        );


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        path1.init(movePower, headingOffset);
        while(opModeInInit()){
            slidesArm.setDegrees(30);
            clawWrist.setWristState(ClawWrist.WristState.NEUTRAL);
            clawWrist.setClawState(ClawWrist.ClawState.CLOSED);
            slidesArm.setInches(0);
            slidesArm.update();
            clawWrist.update();
        }

        while(opModeIsActive()){
            switch(currentState){
                case FIRST:
                    path1.update();
                    if (timeToggle) {//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if (timer.milliseconds() > TimeStamp + 2500) {
                        timeToggle = true;
                        currentState = State.SECOND;
                    }
                    break;
                case SECOND:
                    path1.setMovePower(0);
                    clawWrist.setWristState(ClawWrist.WristState.INTAKE);
                    if (timeToggle) {//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if (timer.milliseconds() > TimeStamp + 1000) {
                        timeToggle = true;

                        path1.setMovePower(1);
                        currentState = State.THIRD;
                    }
                    break;
                case THIRD:

                    path1.update();
                    if (timeToggle) {//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if (timer.milliseconds() > TimeStamp + 2500) {
                        timeToggle = true;
                        path2.init(movePower, headingOffset);
                        currentState = State.THIRD;
                    }
                    break;
            }

            telemetry.addData("pose x", drive.getX());
            telemetry.addData("pose y", drive.getY());
            telemetry.addData("pose r", drive.getR());



            telemetry.addData("segment ", PurePursuitUtil.getSegment());
            telemetry.addData("time ", timer.seconds());
            telemetry.addData("state ", currentState);
            telemetry.update();
            clawWrist.update();
            slidesArm.update();
        }
    }
}
