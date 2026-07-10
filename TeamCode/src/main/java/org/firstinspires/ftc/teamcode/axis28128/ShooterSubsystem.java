package org.firstinspires.ftc.teamcode.axis28128;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Shared shooter / intake / spindexer / turret logic, usable from both
 * TeleOp and Autonomous OpModes. Call update() once per loop() iteration.
 */
@Configurable
public class ShooterSubsystem {

    // === BALL DETECT ===
    public static double BALL_DETECT_THRESHOLD = 65;
    public static int BALL_DETECT_CONSECUTIVE = 3;
    private int closeReadingStreak = 0;
    public boolean ballWasDetected = false;

    // === SHOOTING TIMING ===
    public static double SHOOT_ADVANCE_MS = 450;
    private double nextShootAdvanceTime = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // === SHOOTER SPEEDS ===
    public static double shooterMaxTPS = 6200, shooterMinTPS = 2000;
    public double currentTPS = shooterMaxTPS;

    // === SHOOTER PIDF ===
    public static double kV = 0.00009;
    public static double kS = 0.05;
    public static double kP = 0.0001;
    private static final double GEAR_RATIO = (double) 10 / 16;

    // === TURRET ===
    public static double TURRET_TPR = 873;
    public static double TURRET_TICKS_PER_RADIAN = TURRET_TPR / (2 * Math.PI);
    public static double TURRET_PWR = 0.3;
    public static double TURRET_ANGLE_SIGN = -1;
    public static double TURRET_ANGLE_OFFSET = (Math.PI / 6) + (Math.PI / 18) + (Math.PI / 36);
    public static double TURRET_MIN_ANGLE = 0;
    public static double TURRET_MAX_ANGLE = 0;
    public static int TURRET_TICK_MIN = -390;
    public static int TURRET_TICK_MAX = 500;
    public static final double GOAL_X = 140, GOAL_Y = 140;

    // === SPINDEXER ===
    public double[] spindexerPos = {0.31, 0.55, 0.8, 0.62, 0.36, 0.15};

    public int spinidx = 0;

    // === STATE ===
    public boolean isShooting = false;
    public double measuredDistance = 0;
    public double currentRPM = 0;

    // === HARDWARE ===
    public DcMotorEx shooterMotor;
    public DcMotor transferMotor, turretMotor, intake;
    public Servo spindexerServo, bootKickerServo, shooterServo;
    public DistanceSensor spindexDistance;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor    = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        transferMotor   = hardwareMap.get(DcMotor.class,   "transferMotor");
        turretMotor     = hardwareMap.get(DcMotor.class,   "turretMotor");
        intake          = hardwareMap.get(DcMotor.class,   "intakeMotor");

        spindexerServo  = hardwareMap.get(Servo.class, "spindexerServo");
        bootKickerServo = hardwareMap.get(Servo.class, "transferServo");
        shooterServo    = hardwareMap.get(Servo.class, "shooterServo");

        spindexDistance = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        bootKickerServo.setPosition(0);
        shooterServo.setPosition(0);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer.reset();
    }

    /**
     * Call once per loop. wantsToShoot = true while the shoot command is active
     * (e.g. left bumper held in TeleOp, or "true" during an autonomous shoot phase).
     * intakeIn / intakeOut control the intake motor independently.
     * follower is used to aim the turret at the goal.
     */
    public void update(boolean wantsToShoot, boolean intakeIn, boolean intakeOut, Follower follower) {
        // --- Shooter velocity / RPM ---
        currentRPM = (((-shooterMotor.getVelocity()) / 28) * 60) / GEAR_RATIO;
        double ff = kS + (kV * currentTPS);
        double error = currentTPS - currentRPM;
        double feedback = kP * error;
        double pwr = Math.max(0, ff + feedback);

        // --- Distance sensor ---
        measuredDistance = spindexDistance.getDistance(DistanceUnit.MM);
        isShooting = (currentRPM >= currentTPS);

        // --- Shooter / transfer / turret aiming ---
        if (wantsToShoot) {
            shooterMotor.setPower(pwr);
            transferMotor.setPower(0.1);

            Pose pose = follower.getPose();
            double angleToGoal = Math.atan2(GOAL_Y - pose.getY(), GOAL_X - pose.getX());
            double turretTarget = angleToGoal * TURRET_ANGLE_SIGN + TURRET_ANGLE_OFFSET - pose.getHeading();
            setTurretAngle(turretTarget, TURRET_PWR);

            bootKickerServo.setPosition(0.3); // transfer(true)

            if (isShooting) {
                if (timer.milliseconds() >= nextShootAdvanceTime && spinidx < 7) {
                    spinidx++;
                    nextShootAdvanceTime = timer.milliseconds() + SHOOT_ADVANCE_MS;
                }
            }
        } else {
            shooterMotor.setPower(0);
            transferMotor.setPower(0);
            bootKickerServo.setPosition(0); // transfer(false)

            if (spinidx >= 3) spinidx = 0;
            nextShootAdvanceTime = 0;
        }

        if (!wantsToShoot && isShooting) spinidx = 0;

        // --- Intake ---
        if (intakeIn && spinidx != 3) {
            intake.setPower(-0.8);
        } else if (intakeOut) {
            intake.setPower(0.8);
        } else {
            intake.setPower(0);
        }

        // --- Ball detect / auto-advance spindexer ---
        boolean readingIsClose = measuredDistance <= BALL_DETECT_THRESHOLD;
        if (readingIsClose) closeReadingStreak++;
        else closeReadingStreak = 0;

        boolean ballNearNow = closeReadingStreak >= BALL_DETECT_CONSECUTIVE;
        if (ballNearNow && !ballWasDetected && !wantsToShoot && spinidx < 2) {
            spinidx++;
        }
        ballWasDetected = ballNearNow;

        if (spinidx > 6) spinidx = 0;
        if (spinidx < 0) spinidx = 0;

        spindexerServo.setPosition(spindexerPos[spinidx]);
    }

    /** Manual spindexer nudge (e.g. dpad in TeleOp). */
    public void manualAdvance(int delta) {
        spinidx += delta;
        if (spinidx > 6) spinidx = 0;
        if (spinidx < 0) spinidx = 0;
    }

    /** Toggle between near/far shooting speed. */
    public void toggleSpeed() {
        currentTPS = (currentTPS == shooterMinTPS) ? shooterMaxTPS : shooterMinTPS;
    }

    public void adjustShooterServo(double delta) {
        shooterServo.setPosition(shooterServo.getPosition() + delta);
    }

    // === TURRET HELPERS ===

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
        double delta = normalizeDelta(targetRadians - currentRadians);

        if (Math.abs(delta) < 0.01) {
            turretMotor.setPower(0);
            return;
        }

        double newTarget = currentRadians + delta;
        int targetTicks = (int) (newTarget * TURRET_TICKS_PER_RADIAN);
        if (targetTicks < -490) targetTicks += (int) TURRET_TPR;

        if (targetTicks < TURRET_TICK_MIN || targetTicks > TURRET_TICK_MAX) {
            turretMotor.setPower(0);
            return;
        }

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(pwr);
    }

    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / TURRET_TICKS_PER_RADIAN;
    }
}