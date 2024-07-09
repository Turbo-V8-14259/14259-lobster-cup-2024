package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

public class ClawWrist {
    public enum ClawState {
        OPENLeft,
        OPENRight,
        OPEN,
        CLOSEDLeft,
        CLOSEDRight,
        CLOSED,
        PIXEL_REARRANGE
    }
    public enum WristState {
        INTAKE,
        OUTTAKE,
        NEUTRAL,
        PIXEL_REARRANGE
    }
    public ClawState clawFSM = ClawState.CLOSED;
    public WristState wristFSM = WristState.NEUTRAL;

    private ServoMotorBetter clawLeft;
    private ServoMotorBetter clawRight;
    private ServoMotorBetter wrist;

    public ClawWrist(HardwareMap hardwareMap) {
        clawLeft = new ServoMotorBetter(hardwareMap.get(Servo.class, "c1"));
        clawRight = new ServoMotorBetter(hardwareMap.get(Servo.class, "c2"));

        clawLeft.setLowerBound(0.15);
        clawLeft.setUpperBound(0.53);
        clawRight.setLowerBound(0.87);
        clawRight.setUpperBound(0.49);
        wrist = new ServoMotorBetter(hardwareMap.get(Servo.class, "w"));
        wrist.setLowerBound(1);
        wrist.setUpperBound(.25);
        //bounds
    }
    public ClawWrist.ClawState getStateClaw(){
        return clawFSM;
    }
    public ClawWrist.WristState getStateWrist(){
        return wristFSM;
    }
    private double targetL = 0;
    private double targetR = 0;

    public void setClawState(ClawState state){
        this.clawFSM = state;
        switch (clawFSM){
            case OPENLeft:
                targetL = 0;
                break;
            case OPENRight:
                targetR = 0;
                break;
            case CLOSEDLeft:
                targetL = 1;
                break;
            case CLOSEDRight:
                targetR = 1;
                break;
            case OPEN:
                targetL = 0;
                targetR = 0;
                break;
            case CLOSED:
                targetL = 1;
                targetR = 1;
                break;
        }
    }

    private double targetW = 0;
    public void setWristState(WristState state){
        this.wristFSM = state;
        switch (wristFSM){
            case INTAKE:
                targetW = 0;
                break;
            case NEUTRAL:
                targetW = 1;
                break;
            case OUTTAKE:
                //math
                break;
            case PIXEL_REARRANGE:
                //score position offsetted by sm
                break;
        }
    }
    public void update(){
        clawLeft.setPosition(targetL);
        clawRight.setPosition(targetR);
        wrist.setPosition(targetW);
        clawLeft.update();
        clawRight.update();
        wrist.update();
    }






}
