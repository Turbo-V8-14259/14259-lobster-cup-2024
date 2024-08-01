
package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
import org.firstinspires.ftc.teamcode.hardware.ClawWrist;
import org.firstinspires.ftc.teamcode.hardware.SATest;
import org.firstinspires.ftc.teamcode.vision.pipeline.CameraPipelineBlue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
@Config
public class farBlueAuto extends LinearOpMode {

    enum State {
        PROP,
        PIXEL1,
        PIXELDOWN,
        PIXELRETRACT,
        TRAVEL,
        TRAVELINTERMEDIATE,
        DEPOSIT,
        ARMFLIP,
        SLIDESOUT,
        SCORE,
        SCORERETRACT,
        PARK1,
        PARK2,
        PARK2LAST,
        FINISH
    }
    DT drive;
    boolean timeToggle = true;
    double TimeStamp = 0;
    ElapsedTime timer = new ElapsedTime();
    State currentState;
    public static Pose2d startPose = new Pose2d(-38,60, Math.toRadians(-90));

    public static Pose2d propMiddle = new Pose2d(-38, 50,Math.toRadians(-95));
    public static Pose2d propLeft = new Pose2d(-38, 50,Math.toRadians(-75));
    public static Pose2d propRight = new Pose2d(-38, 50,Math.toRadians(-127.5));
    Pose2d travelIntermediate = new Pose2d(-38, 55,Math.toRadians(-180));
    Pose2d travel = new Pose2d(20, 52,Math.toRadians(-180));
    public static Pose2d boardRight = new Pose2d(38, 25.5,Math.toRadians(-180));
    public static Pose2d boardMiddle = new Pose2d(38, 31,Math.toRadians(-180));
    public static Pose2d boardLeft = new Pose2d(38, 37,Math.toRadians(-180));

    public static Pose2d park1 = new Pose2d(45,55, Math.toRadians(-180));
    public static Pose2d park2first = new Pose2d(37,5, Math.toRadians(-180));
    public static Pose2d park2last = new Pose2d(45,5, Math.toRadians(-180));

    int intakeExtension = 10;
    public static int armAngle = 145;
    public static int pixelExtendsion = 12;
    public static int pixelMiddleExtension =16;
    public static int depositExtension =8;
    public static int randomization=0;//0:left, 1:middle, 2:right
    //if(randomization==0){
    //
    //                    }else if(randomization==1){
    //
    //                    }else{
    //
    //                    }
    private String ObjectDirection;
    private double thresh = CameraPipelineBlue.perThreshold;
    public static String color = "RED";
    public static int delayAction = 750;
    public static int delayRun = 1000;
    public static int delayTravel=3000;
    public static boolean whichPark=true; //true right of backboard, false left of backboard
    ElapsedTime timer2 = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DT(hardwareMap, new Pose2d(startPose.getX(), startPose.getY(), startPose.getHeading()), timer);
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        SATest slidesArm = new SATest(hardwareMap, timer2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        currentState = State.PROP;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        CameraPipelineBlue coneDetectionPipeline = new CameraPipelineBlue(telemetry);
        camera.setPipeline(coneDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(864, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        while(opModeInInit())
        {
            if(gamepad1.left_bumper){//park right of the board
                whichPark=false;
            }
            if(gamepad1.right_bumper){
                whichPark=true;
            }
            if(whichPark){
                telemetry.addData("park", "side of the board");
            }else{
                telemetry.addData("park", "center of the field");
            }
            ObjectDirection = CameraPipelineBlue.randomization(thresh);
            telemetry.addLine(ObjectDirection);

            if (ObjectDirection == "LEFT") {//camera opencv pipeline
                randomization = 0;
            } else if (ObjectDirection == "MIDDLE") {
                randomization = 1;
            } else if (ObjectDirection == "RIGHT") {
                randomization = 2;
            }
            telemetry.addLine("randomization: " + randomization);
            telemetry.update();
            slidesArm.setDegrees(30);
            clawWrist.setWristState(ClawWrist.WristState.NEUTRAL);
            clawWrist.setClawState(ClawWrist.ClawState.CLOSED);
            slidesArm.setInches(0);
            slidesArm.update();
            clawWrist.update();

        }
        camera.stopStreaming();
        waitForStart();

        while(opModeIsActive()){
            switch(currentState){
                case PROP:
                    clawWrist.setWristState(ClawWrist.WristState.INTAKE);
                    if(randomization==0) {
                        drive.lineTo(propLeft.getX(), propLeft.getY(), propLeft.getHeading());

                        if (timeToggle) {//timeToggle starts at true by default
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if (timer.milliseconds() > TimeStamp + delayRun) {
                            timeToggle = true;
                            currentState = State.PIXEL1;

                        }

                    }else if(randomization==1){
                        drive.lineTo(propMiddle.getX(),propMiddle.getY(), propMiddle.getHeading());

                        if(timeToggle){//timeToggle starts at true by default
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if(timer.milliseconds() > TimeStamp + delayRun){
                            timeToggle = true;
                            currentState = State.PIXEL1;

                        }
                    }else{
                        drive.lineTo(propRight.getX(),propRight.getY(), propRight.getHeading());
                        if(timeToggle){//timeToggle starts at true by default
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if(timer.milliseconds() > TimeStamp + delayRun){
                            timeToggle = true;
                            currentState = State.PIXEL1;

                        }
                    }


                    break;
                case PIXEL1:
                    slidesArm.setDegrees(15);

                    if(randomization==1){
                        slidesArm.setInches(pixelMiddleExtension);
                    }else{
                        slidesArm.setInches(pixelExtendsion);
                    }



                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayAction){
                        timeToggle = true;
                        currentState = State.PIXELDOWN;
                    }
                    break;
                case PIXELDOWN:
                    clawWrist.setClawState(ClawWrist.ClawState.OPENLeft);
                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayAction){
                        timeToggle = true;
                        currentState = State.PIXELRETRACT;
                    }
                    break;
                case PIXELRETRACT:

                    slidesArm.setInches(1);
                    clawWrist.setClawState(ClawWrist.ClawState.CLOSED);
                    clawWrist.setWristState(ClawWrist.WristState.NEUTRAL);

                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayAction){
                        timeToggle = true;

                        currentState=State.TRAVELINTERMEDIATE;
                    }


                    break;
                case TRAVELINTERMEDIATE:

                    drive.lineTo(travelIntermediate.getX(), travelIntermediate.getY(), travelIntermediate.getHeading());

                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayRun){
                        timeToggle = true;

                        currentState= State.TRAVEL;
                    }


                    break;
                case TRAVEL:

                    drive.lineTo(travel.getX(), travel.getY(), travel.getHeading());

                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayTravel){
                        timeToggle = true;

                        currentState= State.DEPOSIT;
                    }
                    break;

                case DEPOSIT:
                    if(randomization==0){
                        drive.lineTo(boardLeft.getX(),boardLeft.getY(), boardLeft.getHeading());

                        if(timeToggle){//timeToggle starts at true by default
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if(timer.milliseconds() > TimeStamp + delayRun){
                            timeToggle = true;
                            currentState = State.ARMFLIP;

                        }
                    }else if(randomization==1){
                        drive.lineTo(boardMiddle.getX(),boardMiddle.getY(), boardMiddle.getHeading());

                        if(timeToggle){//timeToggle starts at true by default
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if(timer.milliseconds() > TimeStamp + delayRun){
                            timeToggle = true;
                            currentState = State.ARMFLIP;

                        }
                    }else{
                        drive.lineTo(boardRight.getX(),boardRight.getY(), boardRight.getHeading());

                        if(timeToggle){//timeToggle starts at true by default
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if(timer.milliseconds() > TimeStamp + delayRun){
                            timeToggle = true;
                            currentState = State.ARMFLIP;

                        }
                    }

                    break;
                case ARMFLIP:
                    slidesArm.setDegrees(armAngle);
                    clawWrist.setWristState(ClawWrist.WristState.OUTTAKE);
                    clawWrist.alignBoard(-slidesArm.getCurrentDegrees());
                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayAction){
                        timeToggle = true;
                        currentState = State.SLIDESOUT;

                    }
                    break;
                case SLIDESOUT:
                    slidesArm.setInches(depositExtension);

                    clawWrist.setWristState(ClawWrist.WristState.OUTTAKE);
                    clawWrist.alignBoard(-slidesArm.getCurrentDegrees());
                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayAction){
                        timeToggle = true;
                        currentState = State.SCORE;

                    }
                    break;
                case SCORE:
                    clawWrist.setClawState(ClawWrist.ClawState.OPENRight);
                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayAction){
                        timeToggle = true;
                        currentState=State.SCORERETRACT;

                    }
                    break;
                case SCORERETRACT:
                    clawWrist.setClawState(ClawWrist.ClawState.CLOSED);
                    clawWrist.setWristState(ClawWrist.WristState.NEUTRAL);
                    slidesArm.setInches(0);
                    slidesArm.setDegrees(35);

                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayAction){
                        timeToggle = true;
                        if(whichPark){
                            currentState = State.PARK1;
                        }else{
                            currentState = State.PARK2;
                        }

                    }
                    break;
                case PARK1:
                    drive.lineTo(park1.getX(), park1.getY(), park1.getHeading());

                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayRun){
                        timeToggle = true;
                        currentState = State.FINISH;

                    }

                    break;
                case PARK2:
                    drive.lineTo(park2first.getX(), park2first.getY(), park2first.getHeading());

                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayRun){
                        timeToggle = true;
                        currentState = State.PARK2LAST;

                    }

                    break;
                case PARK2LAST:
                    drive.lineTo(park2last.getX(), park2last.getY(), park2last.getHeading());

                    if(timeToggle){//timeToggle starts at true by default
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + delayRun){
                        timeToggle = true;
                        currentState = State.FINISH;

                    }

                    break;
                case FINISH:
                    break;
            }
            telemetry.addData("state", currentState);
            telemetry.addData("x", drive.getX());
            telemetry.addData("y", drive.getY());
            telemetry.addData("r", drive.getR());
            telemetry.update();
            drive.update();
            slidesArm.update();
            clawWrist.update();
        }
    }
}
