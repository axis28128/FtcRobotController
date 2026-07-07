package org.firstinspires.ftc.teamcode.axis28128.Flywheel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Configurable
@TeleOp(name = "kS Flywheel Tuner")
public class kSTuner extends OpMode {
    Flywheel flywheel = new Flywheel();
    public static double kS = 0.043;
    @Override
    public void init() {
        flywheel.init(hardwareMap);
    }
    @Override
    public void loop() {
        if(gamepad1.dpadUpWasPressed()) kS+=0.001;
        else if(gamepad1.dpadDownWasPressed()) kS-=0.001;
        flywheel.setMotorPower(kS);
        telemetry.addData("kS: ", kS);
        telemetry.addData("RPM: ", flywheel.getRPM());
        telemetry.addData("TPS: ", flywheel.getTPS());


    }
}
