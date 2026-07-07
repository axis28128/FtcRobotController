package org.firstinspires.ftc.teamcode.axis28128.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;
    private double power = 0.8;
    public void init(HardwareMap hw) {
        intakeMotor = hw.get(DcMotor.class, "intakeMotor");
    }
    public double getPower() {return power;}
    public void setPower(double newpower) {
        power = newpower;
    }


}
