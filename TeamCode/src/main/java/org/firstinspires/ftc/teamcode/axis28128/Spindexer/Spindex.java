package org.firstinspires.ftc.teamcode.axis28128.Spindexer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Spindex {
    private Servo spindexer;
    private int idx = 0;
    private double[] positions = {0.24, 0.48, 0.72, 0.53, 0.27, 0};
    public void init(HardwareMap hw) {
        spindexer = hw.get(Servo.class, "spindexerServo");
    }
    public void goBackOne() {
        idx = Math.max(0, idx-1);
    }
    public void goUpOne() {
        idx = Math.min(5, idx+1);
    }
    public void setPos(int index) {
        idx = index;
    }
    public void runSpindexer() {
        //Should be in some void loop
        spindexer.setPosition(positions[idx]);
    }
    public int getIndex() {
        return idx;
    }



}
