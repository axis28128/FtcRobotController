package org.firstinspires.ftc.teamcode.axis28128;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "configuration master")
public class NewSpindex extends OpMode {
    public static double pos = 0;
    public Servo spindexerServo;
    public DistanceSensor distanceSensor;

    @Override
    public void init() {
        spindexerServo = hardwareMap.get(Servo.class, "shooterServo");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        spindexerServo.setPosition(pos);
    }

    @Override
    public void loop() {
        if(gamepad1.dpadUpWasPressed()) pos += 0.01;
        else if (gamepad1.dpadDownWasPressed()) pos -= 0.01;
        telemetry.addData("Spindexer current position: ", pos);
        telemetry.addData("Distance Measured: ", distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.update();
        spindexerServo.setPosition(pos);

    }
}