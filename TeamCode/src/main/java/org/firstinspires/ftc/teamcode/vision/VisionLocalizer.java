package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipeline.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class VisionLocalizer {


    static final double FEET_PER_METER = 3.28084;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private final double fx;
    private final double fy;
    private final double cx;
    private final double cy;
    private final HardwareMap hardwareMap;
    private final String camName;
    private final int id;
    private final float ox;
    private final float oz;
    // UNITS ARE METERS
    double tagsize = 0.0508;
    int numFramesWithoutDetection = 0;


    public VisionLocalizer(double fx, double fy, double cx, double cy, HardwareMap hardwareMap, String camName, int id, float ox, float oz) {
        AprilTagLibrary curLib = AprilTagGameDatabase.getCenterStageTagLibrary();
        this.fx=fx;
        this.fy=fy;
        this.cx=cx;
        this.cy=cy;
        this.hardwareMap=hardwareMap;
        this.camName=camName;
        this.id=id;
        this.ox=ox;
        this.oz=oz;


    }
    public void startStreaming(){
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, camName), id);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    public ArrayList<AprilTagDetection> testFunc() {
        return aprilTagDetectionPipeline.getDetectionsUpdate();
    }
    public void stopCam(){
        camera.stopStreaming();
    }


}
