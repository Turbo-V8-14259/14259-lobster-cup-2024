package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.vision.VisionLocalizer;

@TeleOp
public class testOpMode extends LinearOpMode {
    double fx = 635.906168592f;
    double fy = 635.906168592f;
    double cx = 451.512327554f;
    double cy = 232.379726059f;
    public void runOpMode(){
        VisionLocalizer visionLocalizer = new VisionLocalizer(fx,fy,cx,cy,hardwareMap);
        waitForStart();
        telemetry.setMsTransmissionInterval(50);
    }
}
