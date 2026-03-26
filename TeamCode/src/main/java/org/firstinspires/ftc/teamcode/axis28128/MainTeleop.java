package org.firstinspires.ftc.teamcode.axis28128;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name="Main Teleop Mode")
@Configurable
public class MainTeleop extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    public Supplier<PathChain> pathChain;
    public TelemetryManager telemetryM;
    public static double maxTransferServoPos = 0.8;
    public static double shooterRampUpTimer = 3.1;
    public static double TURRET_TPR = 28 * 3 * 4 * 3.3, TURRET_TICKS_PER_RADIAN = TURRET_TPR / (2 * Math.PI), TURRET_PWR = 0.1;
    public static double TURRET_L_BOUND, TURRET_R_BOUND;
    public DcMotor shooterMotor, transferMotor, turretMotor, intake;
    public CRServo spindex;
    public Servo bootkick, shooterServo;
    public NormalizedColorSensor colorSensor;
    public DistanceSensor distanceMeasure;
    public void transfer(boolean shouldTransfer) {
        if(shouldTransfer) bootkick.setPosition(maxTransferServoPos);
        else bootkick.setPosition(0);
    }
    public void setTurretAngle(double radians, double pwr) {
        //Update turretMotor angle in radians
        //when going out of bounds, need to +2Math.PI or -2Math.PI (full rotation so that cables won't fuck themselves)
        if(turretMotor.getCurrentPosition() + (int)(TURRET_TICKS_PER_RADIAN * radians) > TURRET_L_BOUND) {
            setTurretAngle(radians-(2*Math.PI), pwr);
            return;
        } else if(turretMotor.getCurrentPosition() + (int)(TURRET_TICKS_PER_RADIAN*radians) < TURRET_R_BOUND) {
            setTurretAngle(radians+(2*Math.PI), pwr);
            return;
        }
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition() + (int)(TURRET_TICKS_PER_RADIAN * radians));
        turretMotor.setPower(pwr);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public double getTargetAngle(double rx, double ry, boolean isRed) {
        //calculate targetAngle
        double thx, thy;
        if(isRed) {
            thx = 144 - rx;
        } else {
            //tipa negative, have to test ts not sure daca merge in practice
            thx = -rx;
        }
        thy = 144 - ry;
        return Math.atan2(thy, thx);
    }
    public boolean wasPressed = false, isShooting = false;
    public static ElapsedTime shootSeqTimer = new ElapsedTime();
    @Override
    public void init() {

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");

        //Singular CRServo used for indexer
        //because we are not using a singular servo, we are forced to use sensors to track positions
        spindex = hardwareMap.get(CRServo.class, "spindexerServo");

        //Servos responsible for other components in our systems
        bootkick = hardwareMap.get(Servo.class, "transferServo"); //on / off
        shooterServo = hardwareMap.get(Servo.class, "shooterServo"); //angle
        //need more values for shooter servo, or program it so that 45 deg will be max (1 value, and 0 value means 0 deg)

        //Sensors used in spindexer for tracking and current index position balls inside indexer
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceMeasure = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        //motor behaviors, set servo init positions
        bootkick.setPosition(0);
        shooterServo.setPosition(0);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //pedropathing followers for teleop
        follower = Constants.createFollower(hardwareMap);
        Pose startingPose = new Pose(80, 8, 0);
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }
    @Override
    public void start() {
        follower.startTeleOpDrive();
    }
    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        //shooting sequence logic
        //should update after PIDF flywheel tuner
        if(gamepad1.left_bumper) {
            if(!wasPressed) {
                shootSeqTimer.reset();
                wasPressed = true;
                transfer(true);
            }
            shooterMotor.setPower(0.8);
            if(shootSeqTimer.seconds() >= shooterRampUpTimer) {
                isShooting = true;
                spindex.setPower(0.2);
                transferMotor.setPower(1);
                transfer(true);
            }
        } else {
            isShooting = false;
            transfer(false);
            transferMotor.setPower(0);
            shooterMotor.setPower(0);
            shootSeqTimer.reset();
            wasPressed = true;
        }

        //simple intake logic
        if(gamepad1.right_bumper) intake.setPower(-1);
        else if(gamepad1.y) intake.setPower(1); //edge case when ejecting is needed
        else intake.setPower(0);

        //Logic for indexer for measured distance to be implemented
        //more logic for indexer should be implemented (ballCounter)
        double measuredDistance = distanceMeasure.getDistance(DistanceUnit.MM);
        if(measuredDistance <= 127 && !isShooting) spindex.setPower(0.17);
        else if (!isShooting) spindex.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

        //teleop drive (pedropathing), field-centric
        follower.setTeleOpDrive(-gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false);

        //turret logic with functions, only done for red as it is the only part we have for testing
        //values for turret things
        Pose currPose = follower.getPose();
        double rx = currPose.getX(), ry = currPose.getY(), rh = follower.getHeading();
        setTurretAngle(getTargetAngle(rx, ry,true) - rh, TURRET_PWR);
    }
}