package org.firstinspires.ftc.teamcode.axis28128;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Configurable
@TeleOp(name = "transfer master")
public class TransferTest extends OpMode {
    public static double pos = 0;
    public Servo spindexerServo;
    public DcMotor transferMotor;

    @Override
    public void init() {
        spindexerServo = hardwareMap.get(Servo.class, "transferServo");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
    }

    @Override
    public void loop() {
        telemetry.update();
        if(gamepad1.dpadUpWasPressed()) {
            pos = 0;
        } else if(gamepad1.dpadDownWasPressed()) {
            pos = 0.3;
        }
        if(gamepad1.a) {
            transferMotor.setPower(0.8);
        } else {
            transferMotor.setPower(0);
        }
        telemetry.addData("Kicker Position: ", pos);
        spindexerServo.setPosition(pos);
    }
}