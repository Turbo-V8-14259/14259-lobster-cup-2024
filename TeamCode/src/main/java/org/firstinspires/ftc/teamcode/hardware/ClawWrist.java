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
        CLOSED
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
        wrist.setLowerBound(0);
        wrist.setUpperBound(1);
        //bounds
    }
    public ClawWrist.ClawState getStateClaw(){
        return clawFSM;
    }
    public ClawWrist.WristState getStateWrist(){
        return wristFSM;
    }
    private double targetC = 0;
    public void setClawState(ClawState state){
        this.clawFSM = state;
        switch (clawFSM){
            case OPENLeft:
                targetC = 0;
                break;
            case OPENRight:
                targetC = 1;
                break;
            case CLOSEDLeft:
                targetC = 1;
                break;
        }
    }




}
