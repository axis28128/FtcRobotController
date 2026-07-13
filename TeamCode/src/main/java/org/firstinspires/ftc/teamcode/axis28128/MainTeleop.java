package org.firstinspires.ftc.teamcode.axis28128;

import static java.lang.Math.sqrt;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Main Teleop Mode")
@Configurable
public class MainTeleop extends OpMode {
    // Add fields
    public static double BALL_DETECT_THRESHOLD = 65; // between ball (~40) and gap (~100)
    public static int BALL_DETECT_CONSECUTIVE = 3;    // readings needed to confirm
    private int FarReadingStreak = 0;
    public static double SHOOT_ADVANCE_MS_FAST = 350, SHOOT_ADVANCE_MS_SORTING = 1000; // tune this — time between each ball feed
    private double nextShootAdvanceTime = 0;
    public static double SHOOTER_POS_FAR = 0.7, SHOOTER_POS_CLOSE = 0.3;
    public boolean sorting = true, globalSorting = true;
    private Follower follower;
    public static Pose startingPose;
    public TelemetryManager telemetryM;
    public int trackingTarget = 1;

    // === DRIVE FEEL TUNING (live-tunable in Panels) ===
    // Translation gets priority: rotation is capped to the wheel budget left over
    // after translation's demand, but never below this floor — so steering stays
    // responsive even at full drive speed. Raise for stronger turning while moving.
    public static double DRIVE_MIN_TURN = 0.2;

    // NEW:
    public static double GOAL_RPM_FAR = 3200, GOAL_RPM_CLOSE = 2750, currentTPS = GOAL_RPM_CLOSE;
    public static double GOAL_MIN_CLOSE_RPM = 2600, GOAL_MIN_FAR_RPM = 3100;
    private boolean shootingFar = true; // tracks which preset is active, so we know which kV to use
    public static double TURRET_TPR = 873;
    public static double TURRET_TICkSFar_PER_RADIAN = TURRET_TPR / (2 * Math.PI);
    public static double TURRET_PWR = 0.3;

    public static double TURRET_ANGLE_SIGN = 1;
    public static double TURRET_ANGLE_OFFSET = ( Math.PI / 6 ) + ( Math.PI / 18) + (Math.PI / 36);

    public static double TURRET_MIN_ANGLE = 0;
    public static double TURRET_MAX_ANGLE = 0;

    public static int TURRET_TICK_MIN = -390;
    public static int TURRET_TICK_MAX = 500;
    // === SHOOTER PIDF TUNING ===
    public static double kVFar = 0.00029, kVClose = 0.000278;  // volts (power) per RPM — main feedforward term
    public static double kSFar = 0.043;     // static friction/minimum power to overcome stiction
    public static double kPFar = 0.025;   // proportional correction for RPM error 3200
    //just need to change goalRPM values between 3200 and 2700 for close and far shooting, also kV value for both. also write correct equation for the PIDF loop.

    public DcMotor transferMotor, turretMotor, intake;
    public DcMotorEx shooterMotor;
    public Servo spindexerServo, bootKickerServo, shooterServo;
    public NormalizedColorSensor colorSensor;
    public DistanceSensor spindexDistance;

    // === VISION: turret AprilTag tracking (Decode RED goal, tag ID 24 "RedTarget") ===
    // The C920 is mounted on the turret, rolled 90° (sideways), CAMERA_OFFSET_MM to the
    // RIGHT of the flywheel center. Because of the 90° roll, a tag to the turret's right
    // appears toward the BOTTOM of the image, so the horizontal aim error lives in
    // ftcPose.z (image vertical), not ftcPose.x. See aimTurretFromCamera() for the math.
    public static String  CAMERA_NAME         = "Webcam 1"; // must match the robot config name
    public static int     RED_GOAL_TAG_ID     = 24;         // Decode "RedTarget"
    public static double  CAMERA_OFFSET_MM    = 87.25;       // camera is this far RIGHT of flywheel center
    public static double  CAMERA_AIM_GAIN     = 1.0;        // scales per-loop turret correction; lower (~0.6) if it hunts
    public static boolean USE_MANUAL_EXPOSURE = true;       // reduces motion blur while the turret moves
    public static int     CAMERA_EXPOSURE_MS  = 6;
    public static int     CAMERA_GAIN         = 250;

    private VisionPortal      visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean cameraExposureSet = false;
    private String  cameraInitError   = null;
    private boolean turretUsingCamera = false; // telemetry: camera vs odometry aim
    private double  lastAimBearingDeg = 0;     // telemetry: flywheel->tag horizontal angle

    private boolean isShooting = false;
    private static ElapsedTime currentTimer = new ElapsedTime();

    private double currentShooterRPM = 0;
    private double currentDistance = 0;
    private double distRatioDebug = 0;

    public double[] spindexerPos = {0.31, 0.55, 0.8, 0.62, 0.36, 0.14};
    public char[] spindexerColor = {'P', 'P', 'G'};
    public String[] patterns = {"GPP", "PGP", "PPG"};
    public int patternIdx = 0, obj = 1;

    // BUG FIX #1: lastMeasuredDistance is now only updated inside the timed block,
    // not again at the bottom of loop(). This ensures the ball-detect comparison
    // between consecutive readings is valid.
    public double lastMeasuredDistance = 0, measuredDistance = 0;

    private double timerTarget = 0;
    private final double deltaDistanceSensorReadingsMillis = 170;
    public int spinidx = 0, shotBalls = 0;
    public boolean ballWasDetected = false;

    // TrackSFar whether PIDF needs to be re-applied (only on change, not every frame).

    @Override
    public void init() {
        shooterMotor   = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        transferMotor  = hardwareMap.get(DcMotor.class,   "transferMotor");
        turretMotor    = hardwareMap.get(DcMotor.class,   "turretMotor");
        intake         = hardwareMap.get(DcMotor.class,   "intakeMotor");

        spindexerServo  = hardwareMap.get(Servo.class, "spindexerServo");
        bootKickerServo = hardwareMap.get(Servo.class, "transferServo");
        shooterServo    = hardwareMap.get(Servo.class, "shooterServo");

        colorSensor    = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        spindexDistance = hardwareMap.get(DistanceSensor.class,       "distanceSensor");

        bootKickerServo.setPosition(0);
        shooterServo.setPosition(0);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startingPose = new Pose(80, 8, Math.toRadians(180));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        try {
            initAprilTag();
        } catch (Exception e) {
            aprilTag = null;
            visionPortal = null;
            cameraInitError = e.getMessage();
        }

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        currentTimer.reset();
        timerTarget += deltaDistanceSensorReadingsMillis;
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();
        if (USE_MANUAL_EXPOSURE && !cameraExposureSet && visionPortal != null
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            setManualExposure(CAMERA_EXPOSURE_MS, CAMERA_GAIN);
        }
        AprilTagDetection goalTag = getRedGoalDetection();
        double gearRatio = (double) 10 / 16;
        double RPM = (((-shooterMotor.getVelocity()) / 28) * 60) * gearRatio;
        double kV = shootingFar ? kVFar : kVClose;
        double ff = kSFar + (kV * currentTPS);
        double error = currentTPS - RPM;
        double feedback = kPFar * error;
        double pwr = Math.max(0, ff + feedback);
        measuredDistance = spindexDistance.getDistance(DistanceUnit.MM);
        if(shootingFar) isShooting = (RPM >= GOAL_MIN_FAR_RPM);
        else isShooting = (RPM >= GOAL_MIN_CLOSE_RPM);
        if (gamepad1.left_bumper) {
            shooterMotor.setPower(pwr);
            transferMotor.setPower(0.8);
            if (goalTag != null) {
                aimTurretFromCamera(goalTag);
            } else {
                aimTurretFromOdometry(obj);
            }
            if((nextShootAdvanceTime - currentTimer.milliseconds()) <= SHOOT_ADVANCE_MS_SORTING/2 && globalSorting) {
                transfer(true);
            } else if(nextShootAdvanceTime - currentTimer.milliseconds() > SHOOT_ADVANCE_MS_SORTING / 2 && globalSorting) transfer(false);
            if(!globalSorting) transfer(!globalSorting);
            if (isShooting) {
                if (currentTimer.milliseconds() >= nextShootAdvanceTime && spinidx < 5 && !globalSorting) {
                    transfer(true);
                    spinidx++; spinidx %= 6; spinidx = Math.max(3, spinidx);
                    shotBalls++;
                    nextShootAdvanceTime = currentTimer.milliseconds() + SHOOT_ADVANCE_MS_FAST;
                } else
                if(currentTimer.milliseconds() >= nextShootAdvanceTime && sorting && shotBalls < 3 && globalSorting) {
                    char neededBall = patterns[patternIdx].charAt(shotBalls);
                    if(neededBall == spindexerColor[0]) {
                        spinidx = 5;
                        spindexerColor[0] = 'X';
                    } else if(neededBall == spindexerColor[1]) {
                        spinidx = 4;
                        spindexerColor[1] = 'X';
                    } else if(neededBall == spindexerColor[2]) {
                        spinidx = 3;
                        spindexerColor[2] = 'X';
                    } else {
                        sorting = false;
                        spinidx = 3;
                    }
                    shotBalls++;
                    nextShootAdvanceTime = currentTimer.milliseconds() + SHOOT_ADVANCE_MS_SORTING;
                }


            }
        } else {
            sorting = true;
            shotBalls = 0;
            shooterMotor.setPower(0);
            transferMotor.setPower(0);
            transfer(false);
            if (spinidx > 3) spinidx = 0;
            nextShootAdvanceTime = globalSorting ? SHOOT_ADVANCE_MS_SORTING : SHOOT_ADVANCE_MS_FAST;
        }
        if (!gamepad1.left_bumper && isShooting) spinidx = 0;
        if (gamepad1.right_bumper && spinidx != 3) {
            intake.setPower(-1);
        } else if (gamepad1.y) {
            intake.setPower(0.8);
        } else {
            intake.setPower(0);
        }
        if (gamepad1.dpadRightWasPressed())      spinidx++;
        else if (gamepad1.dpadLeftWasPressed())  spinidx--;
        boolean readingIsFar = measuredDistance <= BALL_DETECT_THRESHOLD;
        if (readingIsFar) {
            FarReadingStreak++;
        } else {
            FarReadingStreak = 0;
        }
        boolean currentBallGreen = false;
        boolean ballNearNow = FarReadingStreak >= BALL_DETECT_CONSECUTIVE;
        if (ballNearNow && !ballWasDetected && !gamepad1.left_bumper && spinidx < 2) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            if(Math.max(colors.green, Math.max(colors.blue, colors.red)) == colors.green) currentBallGreen = true;
            spindexerColor[spinidx] = (currentBallGreen ? 'G' : 'P');
            spinidx++;
        } else if(ballNearNow && !ballWasDetected && !gamepad1.left_bumper && spinidx < 3) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            if(Math.max(colors.green, Math.max(colors.red, colors.blue)) == colors.green) currentBallGreen = true;
            spindexerColor[spinidx] = (currentBallGreen ? 'G' : 'P');
        }
        ballWasDetected = ballNearNow;
        if (spinidx > 6) spinidx = 0;
        if (spinidx < 0) spinidx = 0;
        spindexerServo.setPosition(spindexerPos[spinidx]);


        // === DRIVE ===
        // Translation-priority mixing: stickSFar pass through at full power, and the
        // turn request is capped to the wheel budget translation leaves unused
        // (Pedro grants heading first, so capping it hands priority to translation).
        // The DRIVE_MIN_TURN floor keeps steering alive at full drive speed.
        double forward = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;

        double turnHeadroom = Math.max(DRIVE_MIN_TURN, 1 - (Math.abs(forward) + Math.abs(lateral)));
        turn = Math.max(-turnHeadroom, Math.min(turnHeadroom, turn));

        follower.setTeleOpDrive(forward, lateral, turn, false, Math.toRadians(180));

        // === TURRET ===
        Pose   currPose    = follower.getPose();
        double rx          = currPose.getX();
        double ry          = currPose.getY();
        double robotHeading = follower.getHeading();
        if (gamepad1.xWasPressed()) {
            shootingFar = !shootingFar;
            currentTPS = shootingFar ? GOAL_RPM_FAR : GOAL_RPM_CLOSE;
            shooterServo.setPosition(shootingFar ? SHOOTER_POS_FAR : SHOOTER_POS_CLOSE);
        }
        if(gamepad1.startWasPressed()) patternIdx = (patternIdx+1)%3;
        if(gamepad1.backWasPressed()) globalSorting = !globalSorting;
        // === SHOOTER SERVO ANGLE ===
        if (gamepad1.dpad_up) {
            shooterServo.setPosition(shooterServo.getPosition() + 0.05);
        } else if (gamepad1.dpad_down) {
            shooterServo.setPosition(shooterServo.getPosition() - 0.05);
        }

        // === ALL TELEMETRY ===
        double curVelocity = RPM;
        telemetryM.addData("Shot Preset", shootingFar ? "FAR (3200)" : "CLOSE (2700)");
        telemetryM.addData("Active kV", kV);
        telemetryM.addData("Meas Dist",         measuredDistance);
        telemetryM.addData("Last Meas Dist",     lastMeasuredDistance);
        telemetryM.addData("spinidx",            spinidx);
        telemetryM.addData("=== SHOOTER ===",   "");
        telemetryM.addData("Distance to Goal",  currentDistance);
        telemetryM.addData("Dist Ratio (0-1)",  distRatioDebug);
        telemetryM.addData("Shooter RPM",       currentShooterRPM);
        telemetryM.addData("Shooting Active",   isShooting);
        telemetryM.addData("Current Velocity",  curVelocity);
        telemetryM.addData("Target Velocity",   currentTPS);
        telemetryM.addData("=== TURRET ===",    "");
        telemetryM.addData("TURRET_ANGLE_SIGN", TURRET_ANGLE_SIGN);
        telemetryM.addData("Robot Heading",   Math.toDegrees(robotHeading));
        telemetryM.addData("Turret Current",  Math.toDegrees(getTurretAngle()));
        telemetryM.addData("=== VISION ===",     "");
        telemetryM.addData("Camera",             cameraInitError == null ? "OK" : ("ERR: " + cameraInitError));
        telemetryM.addData("Red Tag Visible", goalTag != null);
        telemetryM.addData("Turret Aim Source",  turretUsingCamera ? "CAMERA" : "ODOMETRY");
        if (goalTag != null) {
            telemetryM.addData("Tag fwd",    goalTag.ftcPose.y);
            telemetryM.addData("Tag right", -goalTag.ftcPose.z);
            telemetryM.addData("Flywheel Aim", lastAimBearingDeg);
        }
        telemetryM.addData("=== POSITION ===",  "");
        telemetryM.addData("Robot rx",  rx);
        telemetryM.addData("Robot ry ", ry);
        telemetryM.addData("Turret Target Pos", turretMotor.getTargetPosition());
        telemetryM.addData("=== PIDF ===",      "");
        telemetryM.update();

        telemetry.addLine("Spindexer & Patterns");
        telemetry.addData("Spindexer Position 1: ", spindexerColor[0]);
        telemetry.addData("Spindexer Position 2: ", spindexerColor[1]);
        telemetry.addData("Spindexer Position 3: ", spindexerColor[2]);
        telemetry.addData("Pattern Shooting: ", patterns[patternIdx]);
        telemetry.addData("Sorting: ", globalSorting);
        telemetry.addData("Shot Preset", shootingFar ? "FAR (3200)" : "CLOSE (2700)");
        telemetry.addData("Active kV", "%.6f", kV);
        telemetry.addData("Meas Dist",         measuredDistance);
        telemetry.addData("Last Meas Dist",     lastMeasuredDistance);
        telemetry.addData("spinidx",            spinidx);
        telemetry.addData("=== SHOOTER ===",   "");
        telemetry.addData("Distance to Goal",  "%.1f units", currentDistance);
        telemetry.addData("Dist Ratio (0-1)",  "%.2f",       distRatioDebug);
        telemetry.addData("Shooter RPM",       "%.3f",       currentShooterRPM);
        telemetry.addData("Shooting Active",   "%s",         isShooting);
        telemetry.addData("Current Velocity",  curVelocity);
        telemetry.addData("Target Velocity",   currentTPS);
        telemetry.addData("=== TURRET ===",    "");
        telemetry.addData("TURRET_ANGLE_SIGN", "%.0f",       TURRET_ANGLE_SIGN);
        telemetry.addData("TURRET_ANGLE_OFFSET","%.2f rad (%.1f deg)", TURRET_ANGLE_OFFSET, Math.toDegrees(TURRET_ANGLE_OFFSET));
        telemetry.addData("Robot Heading",     "%.1f deg",   Math.toDegrees(robotHeading));
        telemetry.addData("Turret Current",    "%.1f deg",   Math.toDegrees(getTurretAngle()));
        telemetry.addData("=== VISION ===",     "");
        telemetry.addData("Camera",             cameraInitError == null ? "OK" : ("ERR: " + cameraInitError));
        telemetry.addData("Red Tag Visible",   "%s", goalTag != null);
        telemetry.addData("Turret Aim Source",  turretUsingCamera ? "CAMERA" : "ODOMETRY");
        if (goalTag != null) {
            telemetry.addData("Tag fwd / right", "%.1f in / %.1f in", goalTag.ftcPose.y, -goalTag.ftcPose.z);
            telemetry.addData("Flywheel Aim",   "%.1f deg", lastAimBearingDeg);
        }
        telemetry.addData("=== POSITION ===",  "");
        telemetry.addData("Robot",             "(%.1f, %.1f)", rx, ry);
        telemetry.addData("Turret Tick Limits","[%d, %d]",   TURRET_TICK_MIN, TURRET_TICK_MAX);
        telemetry.addData("Turret Target Pos", turretMotor.getTargetPosition());
        telemetry.addData("=== PIDF ===",      "");
        telemetry.update();
    }
    public void transfer(boolean shouldTransfer) {
        bootKickerServo.setPosition(shouldTransfer ? 0.2 : 0);
    }

    public double normalizeAngle(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle < 0) angle += 2 * Math.PI;
        return angle;
    }

    public double normalizeDelta(double delta) {
        delta = delta % (2 * Math.PI);
        if (delta > Math.PI)       delta -= 2 * Math.PI;
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

        double newTarget  = currentRadians + delta;
        int    targetTickSFar = (int) (newTarget * TURRET_TICkSFar_PER_RADIAN);
        if (targetTickSFar < -490) targetTickSFar += (int) TURRET_TPR;

        if (targetTickSFar < TURRET_TICK_MIN || targetTickSFar > TURRET_TICK_MAX) {
            turretMotor.setPower(0);
            return;
        }

        turretMotor.setTargetPosition(targetTickSFar);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(pwr);
    }

    public double getDistance(double rx, double ry) {
        double dx = 144 - rx;
        double dy = 144 - ry;
        return sqrt(dx * dx + dy * dy);
    }

    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / TURRET_TICkSFar_PER_RADIAN;
    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        // Decimation 2 keeps a good detection range at a usable frame rate on a C920.
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();
    }
    private AprilTagDetection getRedGoalDetection() {
        if (aprilTag == null) return null;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == RED_GOAL_TAG_ID && d.ftcPose != null) {
                return d;
            }
        }
        return null;
    }
    private void aimTurretFromOdometry(int object) {
        double angleToGoal = 0;
        //object 1 is blue goal, object 2 is red goal, object 3 is obelisk, object 4 is common goal
        if(object == 1) angleToGoal  = Math.atan2(144 - follower.getPose().getY(), -follower.getPose().getX());
        else if(object == 2) angleToGoal = Math.atan2(144 - follower.getPose().getY(), 144-follower.getPose().getX());
        else if(object == 3) angleToGoal = Math.atan2(144 - follower.getPose().getY(), 72-follower.getPose().getX());
        else if(object == 4) angleToGoal = Math.atan2(follower.getPose().getY()+144, 72-follower.getPose().getX());
        double turretTarget = angleToGoal - follower.getPose().getHeading();
        setTurretAngle(turretTarget, TURRET_PWR);
        turretUsingCamera = false;
    }
    private void aimTurretFromCamera(AprilTagDetection tag) {
        double forward  = tag.ftcPose.y;            // inches along the aim axis
        double rightCam = -tag.ftcPose.z;           // inches, + = tag is to the camera's right

        // Move the tag from the camera's frame into the flywheel's frame.
        double offsetIn = CAMERA_OFFSET_MM / 25.4;  // camera is offsetIn to the right of the flywheel
        double rightFly = rightCam + offsetIn;

        // Horizontal angle the flywheel must swing toward the tag (+ = right / CW).
        double aimRight = Math.atan2(rightFly, forward);
        // Turret frame is CCW-positive (matches odometry aim), so invert; apply as a delta.
        double correctionCcw = -aimRight * CAMERA_AIM_GAIN;

        setTurretAngle(getTurretAngle() + correctionCcw, TURRET_PWR);

        turretUsingCamera = true;
        lastAimBearingDeg = Math.toDegrees(aimRight);
    }
    private void setManualExposure(int exposureMs, int gain) {
        if (visionPortal == null) { cameraExposureSet = true; return; }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl == null) { cameraExposureSet = true; return; }
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            return; // let Manual mode settle; apply the values on the next loop
        }
        exposureControl.setExposure((long) exposureMs, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) gainControl.setGain(gain);
        cameraExposureSet = true;
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}