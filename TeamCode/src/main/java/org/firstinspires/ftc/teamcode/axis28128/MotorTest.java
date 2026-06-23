package org.firstinspires.ftc.teamcode.axis28128;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "intake masturbator")
public class MotorTest extends OpMode {
    public DcMotor motor;
    public float pwr = 1;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    @Override
    public void loop() {
        if(gamepad1.aWasPressed()) pwr = -pwr;
        motor.setPower(pwr);
    }
}