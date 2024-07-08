package org.firstinspires.ftc.teamcode.hardware;

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
    private ServoMotorBetter clawLeft;
    private ServoMotorBetter clawRight;
    private ServoMotorBetter wrist;

    public ClawWrist(ServoMotorBetter clawLeft, ServoMotorBetter clawRight, ServoMotorBetter wrist) {
        this.clawLeft = clawLeft;
        this.clawRight = clawRight;
        this.wrist = wrist;
        //bounds
    }


}
