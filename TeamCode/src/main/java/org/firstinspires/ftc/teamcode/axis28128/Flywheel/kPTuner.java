package org.firstinspires.ftc.teamcode.axis28128.Flywheel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "kP Flywheel Tuner (feedforward)")
public class kPTuner extends OpMode {
    Flywheel flywheel = new Flywheel();
    public static double kP = 0;
    public static double goalRPM = 4500;
    @Override
    public void init() {
        flywheel.init(hardwareMap);
    }
    @Override
    public void loop() {
        double ff = (0.000104 * goalRPM) + flywheel.getkS();
        double error = goalRPM - flywheel.getRPM();
        double feedback = error * kP;
        flywheel.setMotorPower(ff + feedback);
        telemetry.addData("kP: ", kP);
        telemetry.addData("Error: ", error);
        telemetry.addData("RPM: ", flywheel.getRPM());
        telemetry.addData("TPS: ", flywheel.getTPS());
    }
}
