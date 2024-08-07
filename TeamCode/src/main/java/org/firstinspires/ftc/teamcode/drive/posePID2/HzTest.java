package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class HzTest extends LinearOpMode {
    double previousTime = 0;
    double currentTime = 0;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()){
            previousTime = currentTime;
            currentTime = timer.nanoseconds()/1000000000.0;
            telemetry.addData("hz ", 1/(currentTime-previousTime));
            telemetry.update();
        }
    }
}
