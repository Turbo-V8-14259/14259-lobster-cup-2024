package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Objects;

@Config
@TeleOp
public class abtest extends LinearOpMode {
    private String ObjectDirection;
    private double thresh = CameraPipelineRed.perThreshold;
    public static String color = "BLUE";



    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        if (Objects.equals(color, "BLUE")){
            CameraPipelineBlue coneDetectionPipeline = new CameraPipelineBlue(telemetry);
            camera.setPipeline(coneDetectionPipeline);
        } else{
            CameraPipelineRed coneDetectionPipeline = new CameraPipelineRed(telemetry);
            camera.setPipeline(coneDetectionPipeline);
        }



        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(864, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        waitForStart();
        while (opModeIsActive()){

            ObjectDirection = CameraPipelineRed.randomization(thresh);
            telemetry.addLine(ObjectDirection);
            telemetry.update();
        }
    }
}
