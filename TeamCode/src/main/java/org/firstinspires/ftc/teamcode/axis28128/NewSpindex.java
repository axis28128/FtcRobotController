package org.firstinspires.ftc.teamcode.axis28128;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.bylazar.configurables.annotations.Configurable;
@Configurable
@TeleOp(name = "configuration master")
public class NewSpindex extends OpMode {
    public static double pos = 0;
    public Servo spindexerServo;
    public DistanceSensor distanceSensor;
    public NormalizedColorSensor colorSensor;

    @Override
    public void init() {
        spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        spindexerServo.setPosition(pos);
    }

    @Override
    public void loop() {
        if(gamepad1.dpadUpWasPressed()) pos += 0.01;
        else if (gamepad1.dpadDownWasPressed()) pos -= 0.01;
        telemetry.addData("Spindexer current position: ", pos);
        telemetry.addData("Distance Measured: ", distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addLine("==COLOR SENSOR ==");
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //returns 4 values

        telemetry.update();
        spindexerServo.setPosition(pos);

    }
}