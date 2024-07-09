package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;


@Config
public class SlidesArm {
    public enum SlideState {
        RETRACTED,
        INTAKE,
        DEPOSIT
    }
    private static final double S_LOWER_BOUND = 0; //number in ticks
    private static final double S_UPPER_BOUND = -1100; //80.25 inches
    private static final double INCHES_TO_TICKS = -34.97131; //number in ticks

    private DcMotorBetter s;
    private DcMotorBetter a;
    private DcMotorEx bl;


    private PID linSlideController;
    public static double targetLinSlidePosition = 0;
    public static double Kp = 0.01, Ki = 0, Kd = 0;

    public SlidesArm(HardwareMap hardwareMap){
        s = new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "s"));
        a = new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "a"));
        bl = hardwareMap.get(DcMotorEx.class, "bl");
//        this.linSlideController = new PID(new PID.Coefficients(Kp, Ki, Kd),
//                () -> (this.bl.getCurrentPosition()) - this.targetLinSlidePosition,
//                factor -> {
//                    this.s.setPower(M.clamp(factor, 1, -1));
//                });
    }

    public double getCurrentPosition() {
        return this.bl.getCurrentPosition();
    }

    public void setInches(double inches) {
        targetLinSlidePosition = (inches * INCHES_TO_TICKS)  / S_UPPER_BOUND; //untested
    }
    public double getCurrentInches() {
        return  getCurrentPosition()/INCHES_TO_TICKS;
    }

    public void update(){
//        linSlideController.update();
//        s.update();
    }

}
