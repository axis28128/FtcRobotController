package org.firstinspires.ftc.teamcode.axis28128;

import static java.lang.Math.sqrt;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Main Teleop Mode")
@Configurable
public class MainTeleop extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    public TelemetryManager telemetryM;

    public static double maxTransferServoPos = 0.69;
    public static double shooterMaxPwr = 0.8;
    public static double shooterMinPwr = 0.4;
    public static double shooterMaxTPS = 2000, shooterMinTPS = 1680, currentTPS = shooterMaxTPS; //ticks actually
    public static double F = 13, P = 0.7;

    // === SHOOTER DISTANCE SETTINGS ===
    public static double SHOOTER_DIST_MIN = 20;
    public static double SHOOTER_DIST_MAX = 100;
    public static double SHOOTER_ANGLE_MIN = 0;
    public static double SHOOTER_ANGLE_MAX = 0;

    public static double TURRET_TPR = 873;
    public static double TURRET_TICKS_PER_RADIAN = TURRET_TPR / (2 * Math.PI);
    public static double TURRET_PWR = 0.3;

    // === TURRET OFFSETS (TUNE THESE) ===
    public static double TURRET_ANGLE_SIGN = -1;
    public static double TURRET_ANGLE_OFFSET = 0;

    public static double TURRET_MIN_ANGLE = 0;
    public static double TURRET_MAX_ANGLE = 0;

    // === TURRET TICK LIMITS ===
    // The turret will only track when the calculated target is within this range.
    // If the target falls outside [-390, 500] ticks, the motor is stopped.
    public static int TURRET_TICK_MIN = -390;
    public static int TURRET_TICK_MAX = 500;

    public DcMotor transferMotor, turretMotor, intake;
    public DcMotorEx shooterMotor;
    public CRServo spindex;
    public Servo bootkick, shooterServo;
    public NormalizedColorSensor colorSensor;
    public DistanceSensor distanceMeasure;
    private boolean isShooting = false;

    // Current calculated values for telemetry
    private double currentShooterRPM = 0;
    private double currentDistance = 0;
    private double distRatioDebug = 0;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");

        spindex = hardwareMap.get(CRServo.class, "spindexerServo");

        bootkick = hardwareMap.get(Servo.class, "transferServo");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceMeasure = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        bootkick.setPosition(0);
        shooterServo.setPosition(0);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        startingPose = new Pose(80, 8, 0);
        follower = Constants.createFollower(hardwareMap);
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
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        isShooting = (Math.abs(shooterMotor.getVelocity())  >= currentTPS);
        //REVISED SHOOTING LOGIC BECAUSE NOW WE HAVE ENCODER ON SHOOTER MOTOR
        if (gamepad1.left_bumper) {
            shooterMotor.setVelocity(currentTPS);
            if (isShooting) {
                spindex.setPower(0.3);
                transferMotor.setPower(1);
                transfer(true);
            } else {
                spindex.setPower(0);
                transferMotor.setPower(0);
                transfer(false);
            }
        } else {
            shooterMotor.setVelocity(0);
            transferMotor.setPower(0);
            transfer(false);
        }
        if(!gamepad1.left_bumper && isShooting) spindex.setPower(0);

        // === INTAKE LOGIC ===
        if (gamepad1.right_bumper) {
            intake.setPower(-1);
        } else if (gamepad1.y) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        // === SPINDEXER LOGIC ===
        double measuredDistance = distanceMeasure.getDistance(DistanceUnit.MM);
        if (measuredDistance <= 127 && !isShooting) {
            spindex.setPower(0.17);
        } else if (!isShooting) {
            spindex.setPower(gamepad1.left_trigger - gamepad1.right_trigger/Math.PI);
        }

        // === TELEOP DRIVE ===
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        // === TURRET LOGIC ===
        Pose currPose = follower.getPose();
        double rx = currPose.getX();
        double ry = currPose.getY();
        double robotHeading = follower.getHeading();
        double angleToGoal = Math.atan2(133 - ry, 133 - rx);
        double turretTarget = angleToGoal * TURRET_ANGLE_SIGN + TURRET_ANGLE_OFFSET + robotHeading;
        setTurretAngle(turretTarget, TURRET_PWR);

        // === SHOOTER RPM BASED ON DISTANCE ===
        currentDistance = getDistance(rx, ry);
        double clampedDist = currentDistance;
        if (clampedDist < SHOOTER_DIST_MIN) clampedDist = SHOOTER_DIST_MIN;
        if (clampedDist > SHOOTER_DIST_MAX) clampedDist = SHOOTER_DIST_MAX;
        double range = SHOOTER_DIST_MAX - SHOOTER_DIST_MIN;
        if (range > 0) {
            distRatioDebug = (clampedDist - SHOOTER_DIST_MIN) / range;
        } else {
            distRatioDebug = 0;
        }

        currentShooterRPM = shooterMinPwr + distRatioDebug * (shooterMaxPwr - shooterMinPwr);

        double targetServoAngle = SHOOTER_ANGLE_MIN + distRatioDebug * (SHOOTER_ANGLE_MAX - SHOOTER_ANGLE_MIN);

        // === TELEMETRY ===
        int currentTurretTicks = turretMotor.getCurrentPosition();
        boolean turretInRange = (currentTurretTicks >= TURRET_TICK_MIN && currentTurretTicks <= TURRET_TICK_MAX);

        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("Distance to Goal", "%.1f units", currentDistance);
        telemetry.addData("Clamped Dist", "%.1f", clampedDist);
        telemetry.addData("Dist Ratio (0-1)", "%.2f", distRatioDebug);
        telemetry.addData("Shooter RPM", "%.3f", currentShooterRPM);
        telemetry.addData("Servo Angle", "%.2f", targetServoAngle);
        telemetry.addData("Shooting Active", "%s", isShooting);
        telemetry.addData("=== TURRET ===", "");
        telemetry.addData("TURRET_ANGLE_SIGN", "%.0f", TURRET_ANGLE_SIGN);
        telemetry.addData("TURRET_ANGLE_OFFSET", "%.2f rad (%.1f deg)", TURRET_ANGLE_OFFSET, Math.toDegrees(TURRET_ANGLE_OFFSET));
        telemetry.addData("Angle to Goal", "%.1f deg", Math.toDegrees(angleToGoal));
        telemetry.addData("Robot Heading", "%.1f deg", Math.toDegrees(robotHeading));
        telemetry.addData("Turret Target", "%.1f deg", Math.toDegrees(turretTarget));
        telemetry.addData("Turret Current", "%.1f deg", Math.toDegrees(getTurretAngle()));
        telemetry.addData("Turret Tracking", turretInRange ? "ACTIVE" : "OUT OF RANGE - STOPPED");
        telemetry.addData("=== POSITION ===", "");
        telemetry.addData("Robot", "(%.1f, %.1f)", rx, ry);
        telemetry.addData("Turret Ticks", currentTurretTicks);
        telemetry.addData("Turret Tick Limits", "[%d, %d]", TURRET_TICK_MIN, TURRET_TICK_MAX);
        telemetry.addData("Turret Target Pos", turretMotor.getTargetPosition());
        telemetry.addData("=== SHOOTER PID COEFFICIENTS ===", "");
        telemetry.addData("P Value: ", P);
        telemetry.addData("F Value: ", F);

        if (gamepad1.xWasPressed()) {
            if (currentTPS == shooterMinTPS) {
                currentTPS = shooterMaxTPS;
            } else {
                currentTPS = shooterMinTPS;
            }
        }

        if (gamepad1.dpad_up) {
            shooterServo.setPosition(shooterServo.getPosition() + 0.05);
        } else if (gamepad1.dpad_down) {
            shooterServo.setPosition(shooterServo.getPosition() - 0.05);
        }

        double curVelocity = shooterMotor.getVelocity();

        telemetry.addData("Current Velocity: ", curVelocity);
        telemetry.addData("Target Velocity: ", currentTPS);
        telemetry.update();
    }

    // === HELPER METHODS ===

    public void transfer(boolean shouldTransfer) {
        if (shouldTransfer) bootkick.setPosition(maxTransferServoPos);
        else bootkick.setPosition(0);
    }

    public double normalizeAngle(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle < 0) angle += 2 * Math.PI;
        return angle;
    }

    public double normalizeDelta(double delta) {
        delta = delta % (2 * Math.PI);
        if (delta > Math.PI) delta -= 2 * Math.PI;
        else if (delta <= -Math.PI) delta += 2 * Math.PI;
        return delta;
    }

    public void setTurretAngle(double targetRadians, double pwr) {
        targetRadians = normalizeAngle(targetRadians);

        if (TURRET_MAX_ANGLE > TURRET_MIN_ANGLE) {
            targetRadians = Math.max(TURRET_MIN_ANGLE, Math.min(TURRET_MAX_ANGLE, targetRadians));
        }

        double currentRadians = getTurretAngle();
        double delta = targetRadians - currentRadians;
        delta = normalizeDelta(delta);

        if (Math.abs(delta) < 0.01) {
            turretMotor.setPower(0);
            return;
        }

        double newTarget = currentRadians + delta;
        int targetTicks = (int) (newTarget * TURRET_TICKS_PER_RADIAN);
        if (targetTicks < -490) targetTicks += (int) TURRET_TPR;

        // === TICK RANGE GATE ===
        // Only track if the computed target ticks fall within the allowed zone.
        // If out of range, stop the motor and do not command a new position.
        if (targetTicks < TURRET_TICK_MIN || targetTicks > TURRET_TICK_MAX) {
            turretMotor.setPower(0);
            return;
        }

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(pwr);
    }


    public double getDistance(double rx, double ry) {
        double dx = 133 - rx;
        double dy = 133 - ry;
        return sqrt(dx * dx + dy * dy);
    }
    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / TURRET_TICKS_PER_RADIAN;
    }
}