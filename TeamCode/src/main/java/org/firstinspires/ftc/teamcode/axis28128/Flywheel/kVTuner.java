package org.firstinspires.ftc.teamcode.axis28128.Flywheel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "kV Flywheel Tuner (feedforward)")
public class kVTuner extends OpMode {
    Flywheel flywheel = new Flywheel();
    public static double kV = 0.0001;
    public static double goalRPM = 2700;
    @Override
    public void init() {
        flywheel.init(hardwareMap);
    }
    @Override
    public void loop() {
        double pwr = (kV * goalRPM) + flywheel.getkS();
        flywheel.setMotorPower(pwr);
        if(gamepad1.dpadUpWasPressed()) kV+= 0.000001;
        else if(gamepad1.dpadDownWasPressed()) kV -= 0.000001;
        telemetry.addData("kV: ", kV);
        telemetry.addData("RPM: ", flywheel.getRPM());
        telemetry.addData("TPS: ", flywheel.getTPS());


    }
}
