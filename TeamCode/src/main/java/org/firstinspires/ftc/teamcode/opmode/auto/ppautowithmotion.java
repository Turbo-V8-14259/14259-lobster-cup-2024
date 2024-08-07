package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hardware.ClawWrist;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.path.PurePursuit;
//import org.firstinspires.ftc.teamcode.drive.path.PurePursuitPath;
import org.firstinspires.ftc.teamcode.drive.path.PurePursuitUtil;
import org.firstinspires.ftc.teamcode.drive.posePID2.NewDT;
import org.firstinspires.ftc.teamcode.hardware.SATest;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class ppautowithmotion extends LinearOpMode {

    enum PathState {
        PROP,
        PIXEL1,
        PIXELDOWN,
        PIXELRETRACT,
        PARK,
        ONE,
        TWO,
        IDLE,
    }
    enum ActionState{
        IDLE,
        ACTION_1,
        ACTION_2,
        ACTION_3,
        ACTION_4,
        ACTION_5,
    }
    NewDT drive;
    PurePursuit purePursuit;
    ArrayList<Pose2d> path1;
    ArrayList<Pose2d> path2;
    boolean timeToggle = true;
    double TimeStamp = 0;
    ElapsedTime timer2 = new ElapsedTime();
    double previousTime = 0;
    double currentTime = 0;
    ElapsedTime timer = new ElapsedTime();
    public static double movePower =0.5;
    double headingOffset=0;
    //moveRadius/headingRadus can't be greater than radius of path segment
    public static double moveRadius = 5;
    public static double headingRadius = 5;
    public static double dist=30;
    PathState pathState = PathState.ONE;

    ActionState actionState = ActionState.ACTION_1;
    ElapsedTime timerArm = new ElapsedTime();
    public static Pose2d propRight = new Pose2d(12, -50);
    private Pose2d startPose = new Pose2d(9,-60, Math.toRadians(90));
    private Pose2d board = new Pose2d(30,-40);
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NewDT(hardwareMap, startPose, timer);
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        SATest slidesArm = new SATest(hardwareMap, timerArm);


        purePursuit = new PurePursuit(drive, moveRadius, headingRadius, movePower);

        path1 = new ArrayList<>();
        path1.add(startPose);
        path1.add(propRight);



        path2 = new ArrayList<>();
        path2.add(propRight);
        path2.add(board);

        PurePursuitUtil.updateMoveSegment(1);
        PurePursuitUtil.updateHeadingSegment(1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        clawWrist.setWristState(ClawWrist.WristState.NEUTRAL);
        clawWrist.setClawState(ClawWrist.ClawState.CLOSED);

        clawWrist.update();
        waitForStart();
        while(opModeIsActive()){

            switch(actionState){
                case IDLE:
                    break;
                case ACTION_1:

                    clawWrist.setWristState(ClawWrist.WristState.INTAKE);
                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + 500){
                        actionState = ActionState.ACTION_2;

                        timeToggle = true;

                    }

                    break;
                case ACTION_2:
                    slidesArm.setDegrees(15);
                    slidesArm.setInches(10);
                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + 500){
                        timeToggle = true;
                        actionState = ActionState.ACTION_3;

                    }

                    break;
                case ACTION_3:
                    clawWrist.setClawState(ClawWrist.ClawState.OPENLeft);
                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + 500){
                        timeToggle = true;
                        actionState = ActionState.ACTION_4;

                    }

                    break;
                case ACTION_4:
                    slidesArm.setInches(1);
                    clawWrist.setClawState(ClawWrist.ClawState.CLOSED);
                    clawWrist.setWristState(ClawWrist.WristState.NEUTRAL);
                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + 500){
                        timeToggle = true;
                        pathState = PathState.TWO;

                    }

                    break;
            }
            switch(pathState) {
                case IDLE:
                    purePursuit.followPath(false,path1, 0);
                    break;
                case ONE:
                    PurePursuitUtil.setPathLength(purePursuit.getPathLength());
                    purePursuit.followPath(false,path1,0);
                    if(drive.withinPoint(path1.get(path1.size()-1),2)){
                        actionState = ActionState.ACTION_1;
                        pathState = PathState.IDLE;
                    }
                    break;
                case TWO:
                    PurePursuitUtil.setPathLength(purePursuit.getPathLength());
                    purePursuit.followPath(true,path2, 90);

                    break;

            }
            previousTime = currentTime;
            currentTime = timer2.nanoseconds()/1000000000.0;
            telemetry.addData("hz ", 1/(currentTime-previousTime));
            telemetry.addData("pose x", drive.getX());
            telemetry.addData("pose y", drive.getY());
            telemetry.addData("pose r", drive.getR());
            telemetry.addData("timer shit", timer.milliseconds()-TimeStamp);
            telemetry.addData("actionstate", actionState);
            telemetry.addData("pathstate", pathState);
            telemetry.addData("path length", purePursuit.getPathLength());
            telemetry.addData("move segment ", PurePursuitUtil.getMoveSegment());
            telemetry.addData("heading segment ", PurePursuitUtil.getHeadingSegment());
            telemetry.update();
            clawWrist.update();
            slidesArm.update();

        }



    }
}
