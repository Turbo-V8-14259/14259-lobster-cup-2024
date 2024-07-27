package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.vision.pipeline.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


public class VisionLocalizer {
    public static int invX = 1;
    public static int invZ = 1;
    final double FEET_PER_METER = 3.28084;
    private final double fx;
    private final double fy;
    private final double cx;
    private final double cy;
    private final HardwareMap hardwareMap;
    private final String camName;
    private final int id;
    private final double ox;
    private final double oz;

    double t1;
    double t2;
    double t3;
    double c;
    double tx;
    double tz;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double kx;
    double kz;
    VectorF tagPos;
    Orientation tagOrientation;
    AprilTagLibrary curLib = AprilTagGameDatabase.getCurrentGameTagLibrary();
    ArrayList<AprilTagDetection> detections;
    double tagsize = 0.0508;


    public VisionLocalizer(double fx, double fy, double cx, double cy, HardwareMap hardwareMap, String camName, int id, float ox, float oz, int invX, int invZ) {
        AprilTagLibrary curLib = AprilTagGameDatabase.getCurrentGameTagLibrary();

        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.hardwareMap = hardwareMap;
        this.camName = camName;
        this.id = id;
        this.ox = ox / FEET_PER_METER;
        this.oz = oz / FEET_PER_METER;
        this.invX = invX;
        this.invZ = invZ;


    }

    public void startStreaming() {
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

    public Pose2d getGlobalPos() {
        detections = aprilTagDetectionPipeline.getLatestDetections();
        if (!(detections == null || detections.isEmpty())) {
            for (AprilTagDetection detection : detections) {
                try {
                    tagPos = curLib.lookupTag(detection.id).fieldPosition;
                } catch (NullPointerException ignored) {

                }
                tagOrientation = curLib.lookupTag(detection.id).fieldOrientation.toOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                kx = detection.pose.x + (0.064 * invX);
                kz = detection.pose.z + (0.2 * invZ);
                t1 = (Math.PI / 2) - Math.atan2(kx, kz);

                t2 = Math.toRadians(rot.firstAngle);

                t3 = Math.toRadians(180)-(t1 + t2);

                c = Math.sqrt(Math.pow((kx), 2) + Math.pow((kz), 2));

                tx = Math.cos(t3) * c;

                tz = Math.sin(t3) * c;
            }
            return new Pose2d(tx, tz);

        }
        return null;
    }

    public void stopCam() {
        camera.stopStreaming();
    }


}
