package org.firstinspires.ftc.teamcode.axis28128.Flywheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    private DcMotorEx shooterMotor;
    private double gearRatio = (double) 10 /16;
    private double encoderCPM = 28;
    private double kV = 0.000104, kS = 0.1, kP;
    public void init(HardwareMap hw) {
        shooterMotor = hw.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setMotorPower(double pwr) {
        shooterMotor.setPower(pwr);
    }
    public double getTPS() {
        return -shooterMotor.getVelocity();
    }
    public double getRPM() {
        return ((getTPS()/encoderCPM) * 60) / gearRatio;
    }
    public double getkS() {
        return kS;
    }




}
