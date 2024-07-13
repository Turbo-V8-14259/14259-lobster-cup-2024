package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class testOpMode extends LinearOpMode {
    ArrayList<AprilTagDetection> detections;
    ArrayList<AprilTagDetection> detections1;
    double fx = 635.906168592f;
    double fy = 635.906168592f;
    double cx = 451.512327554f;
    double cy = 232.379726059f;

    public void runOpMode() {
        List myPortalsList;

        myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        int Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false)).intValue();
        int Portal_2_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false)).intValue();
        VisionLocalizer visionLocalizer = new VisionLocalizer(fx, fy, cx, cy, hardwareMap, "Webcam 1", Portal_1_View_ID, 2.5f,8f);
        VisionLocalizer visionLocalizer1 = new VisionLocalizer(fx, fy, cx, cy, hardwareMap, "Webcam 2", Portal_2_View_ID, 2.5f,8f);
        visionLocalizer.startStreaming();
        waitForStart();
        telemetry.setMsTransmissionInterval(50);
        telemetry.setAutoClear(true);
        sleep(5000);
        visionLocalizer.stopCam();
        visionLocalizer1.startStreaming();
        while (opModeIsActive() && !isStopRequested()) {
            detections = visionLocalizer.testFunc();
            detections1 = visionLocalizer1.testFunc();
            if (detections != null) {
                if ((detections.isEmpty())) {

                } else {
                    for (AprilTagDetection detection : detections) {
                        telemetry.addLine(detection.id + "");
                    }
                    for (AprilTagDetection detection : detections1) {
                        telemetry.addLine(detection.id + "");
                    }
                }
            }
            telemetry.update();
        }
    }
}
