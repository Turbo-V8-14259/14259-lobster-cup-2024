package org.firstinspires.ftc.teamcode.vision;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class testOpMode extends LinearOpMode {
    int Portal_1_View_ID;
    int Portal_2_View_ID;

    List<Integer> myPortalsList;

    ArrayList<AprilTagDetection> detections;
    double fx = 814.513f;
    double fy = 814.513f;
    double cx = 397.286f;
    double cy = 291.207f;
    public static int invX=1;
    public static int invZ=0;
    double kx;
    double kz;
    static final double FEET_PER_METER = 3.28084;
    VectorF tagPos;
    Orientation tagOrientation;
    AprilTagLibrary curLib = AprilTagGameDatabase.getCurrentGameTagLibrary();

    public void runOpMode() {
        //for dual cameras
        /*
        myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        Portal_1_View_ID = (int) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false);
        Portal_2_View_ID = (int) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false);*/
        Portal_1_View_ID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionLocalizer visionLocalizer = new VisionLocalizer(fx, fy, cx, cy, hardwareMap, "Webcam 1", Portal_1_View_ID, 2.5f,8f);
        visionLocalizer.startStreaming();
        waitForStart();
        telemetry.setMsTransmissionInterval(50);
        telemetry.setAutoClear(true);
        while (opModeIsActive() && !isStopRequested()) {
            detections = visionLocalizer.testFunc();
            if (detections != null) {
                if ((detections.isEmpty())) {

                } else {
                    for (AprilTagDetection detection : detections) {
                        tagPos = curLib.lookupTag(detection.id).fieldPosition;
                        tagOrientation = curLib.lookupTag(detection.id).fieldOrientation.toOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                        kx=detection.pose.x+(0.064*invX);
                        kz=detection.pose.z+(0.2*invZ);
                        double t1 = (Math.PI / 2) - Math.atan2(kz, kz);

                        double t2 = Math.toRadians(rot.firstAngle);

                        double t3 = (t1 + t2);

                        double c = Math.sqrt(Math.pow((kx), 2) + Math.pow((kz), 2));

                        double tx = Math.cos(t3) * c;

                        double tz = Math.sin(t3) * c;


                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
                        telemetry.addLine(String.format("Bearing: %.2f degrees", Math.atan(detection.pose.x / detection.pose.z) * (180 / Math.PI)));
                        telemetry.addLine(String.format("Elevation: %.2f degrees", Math.atan(detection.pose.y / detection.pose.z) * (180 / Math.PI)));
                        telemetry.addLine("True x:" + tx);
                        telemetry.addLine("True z:" + tz);
                    }

                }
            }
            telemetry.update();
        }
    }
}
