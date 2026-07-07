package org.firstinspires.ftc.teamcode.axis28128.Flywheel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.axis28128.Spindexer.Spindex;

@Configurable
@TeleOp(name = "kP Flywheel Tuner (proportional)")
public class kPTuner extends OpMode {
    Flywheel flywheel = new Flywheel();
    Spindex spindexer = new Spindex();
    public static double kP = 0.011;
    public static double goalRPM = 2700;
    @Override
    public void init() {
        flywheel.init(hardwareMap);
        spindexer.init(hardwareMap);
    }
    @Override
    public void loop() {
        if(gamepad1.dpadUpWasPressed()) spindexer.goUpOne();
        else if(gamepad1.dpadDownWasPressed()) spindexer.goBackOne();
        double ff = (flywheel.getkV() * goalRPM) + flywheel.getkS();
        double error = goalRPM - flywheel.getRPM();
        double feedback = error * kP;
        flywheel.setMotorPower(ff + feedback);
        telemetry.addData("kP: ", kP);
        telemetry.addData("Error: ", error);
        telemetry.addData("RPM: ", flywheel.getRPM());
        telemetry.addData("TPS: ", flywheel.getTPS());
        spindexer.runSpindexer();
    }
}
