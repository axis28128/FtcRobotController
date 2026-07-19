package org.firstinspires.ftc.teamcode.axis28128;
// commit 00dc427
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
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
import com.pedropathing.util.Timer;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "New Close Red Test")
@Configurable
public class CloseRedTest extends OpMode {
    public PathChain shootPositionChain, endPositionChain;
    private static final Pose startingPose = new Pose(120, 121.4, Math.toRadians(216));
    public static Pose endPosition = new Pose(85, 105, Math.toRadians(180)); // easily changeable end position
    public enum PathState {
        DRIVE_START_POS_SHOOT_POS,
        SHOOT_PRELOAD,
        SHOOT_POS,
        DRIVE_TO_END_POS,
        IDLE;
    }
    public PathState pathstate;
    private Timer pathTimer, opModeTimer;
    private boolean endPathStarted = false;

    // Add fields
    public static double BALL_DETECT_THRESHOLD = 65; // between ball (~40) and gap (~105)
    public static int BALL_DETECT_CONSECUTIVE = 7;    // readings needed to confirm
    private int FarReadingStreak = 0;
    public static double SHOOT_ADVANCE_MS_FAST = 400, SHOOT_ADVANCE_MS_SORTING = 1050; // tune this — time between each ball feed
    private double nextShootAdvanceTime = 0;
    public static double SHOOTER_POS_FAR = 0.7, SHOOTER_POS_CLOSE = 0.3;
    public boolean sorting = false, globalSorting = false;
    private Follower follower;

    public TelemetryManager telemetryM;
    //EVERYTHING THAT FOR SHOOTER RPM LOGIC
    public static double GOAL_RPM_FAR = 3200, GOAL_RPM_CLOSE = 2500, currentTPS = GOAL_RPM_CLOSE;
    public static double GOAL_MIN_CLOSE_RPM = 2400, GOAL_MIN_FAR_RPM = 3105;
    private boolean shootingFar = false; // tracks which preset is active, so we know which kV to use
    public static double TURRET_TPR = 873;
    public static double TURRET_TICkSFar_PER_RADIAN = TURRET_TPR / (2 * Math.PI);
    public static double TURRET_PWR = 0.3;


    public static double TURRET_ANGLE_SIGN = 1;
    public static double TURRET_ANGLE_OFFSET = ( Math.PI / 6 ) + ( Math.PI / 18) + (Math.PI / 36);

    public static double TURRET_MIN_ANGLE = 0;
    public static double TURRET_MAX_ANGLE = 0;

    public static int TURRET_TICK_MIN = -390;
    public static int TURRET_TICK_MAX = 500;
    //PIDF FOR FLYWHEEL
    public static double kVFar = 0.00029, kVClose = 0.000278;  // volts (power) per RPM — main feedforward term
    public static double kSFar = 0.043;     // static friction/minimum power to overcome stiction
    public static double kPFar = 0.025;   // proportional correction for RPM error 3200
    //just need to change goalRPM values between 3200 and 2700 for close and far shooting, also kV value for both. also write correct equation for the PIDF loop.

    public DcMotor transferMotor, turretMotor, intake;
    public DcMotorEx shooterMotor;
    public Servo spindexerServo, bootKickerServo, shooterServo;
    public NormalizedColorSensor colorSensor;
    public DistanceSensor spindexDistance;
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
    private boolean turretUsingCamera = false;
    private double  lastAimBearingDeg = 0;

    private boolean isShooting = false;
    private static ElapsedTime currentTimer = new ElapsedTime();

    public double[] spindexerPos = {0.2, 0.44, 0.68, 0.51, 0.25, 0.02};
    public double measuredDistance = 0;
    public int spinidx = 2, shotBalls = 0, intakedBalls = 3;
    public boolean ballWasDetected = false;

    @Override
    public void init() {
        pathstate = PathState.DRIVE_START_POS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
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
        shooterServo.setPosition(0.3);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
        try {
            initAprilTag();
        } catch (Exception e) {
            aprilTag = null;
            visionPortal = null;
            cameraInitError = e.getMessage();
        }

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        currentTimer.reset();
        buildPaths();
        follower.setPose(startingPose);
    }
    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathstate);
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();
        telemetry.update();
        boolean readingIsFar = measuredDistance <= BALL_DETECT_THRESHOLD;
        if (readingIsFar) {
            FarReadingStreak++;
        } else {
            FarReadingStreak = 0;
        }
        boolean ballNearNow = FarReadingStreak >= BALL_DETECT_CONSECUTIVE;
        if (ballNearNow && !ballWasDetected && spinidx < 2 && pathstate != PathState.SHOOT_POS && pathstate != PathState.SHOOT_PRELOAD) {
            spinidx++;
            intakedBalls++;
        }
        else if(ballNearNow && !ballWasDetected) intakedBalls++;
        ballWasDetected = ballNearNow;
        if (spinidx > 6) spinidx = 0;
        if (spinidx < 0) spinidx = 0;
        spindexerServo.setPosition(spindexerPos[spinidx]);

        statePathUpdate();
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
    private void aimTurretFromOdometry(int object) {
        double angleToGoal = 0;
        //object 1 is blue goal, object 2 is red goal, object 3 is obelisk, object 4 is common goal
        if(object == 1) angleToGoal  = Math.atan2(144 - follower.getPose().getY(), -follower.getPose().getX());
        else if(object == 2) angleToGoal = Math.atan2(125 - follower.getPose().getY(), 125 - follower.getPose().getX());
        else if(object == 3) angleToGoal = Math.atan2(144 - follower.getPose().getY(), 72-follower.getPose().getX());
        else if(object == 4) angleToGoal = Math.atan2(-follower.getPose().getY() - 144, 72-follower.getPose().getX());
        double turretTarget = angleToGoal - follower.getPose().getHeading();
        setTurretAngle(turretTarget, TURRET_PWR);
        turretUsingCamera = false;
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    public void buildPaths() {
        shootPositionChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.563, 121.422),
                                new Pose(85.404, 105.630)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(0))
                .build();

        endPositionChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                new Pose(endPosition.getX(), endPosition.getY())
                        )
                )
                .setConstantHeadingInterpolation(endPosition.getHeading())
                .build();
    }
    public void statePathUpdate() {
        switch(pathstate) {
            case DRIVE_START_POS_SHOOT_POS: {
                follower.followPath(shootPositionChain, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            }
            case SHOOT_PRELOAD: {
                if (!follower.isBusy()) {
                    double gearRatio = (double) 10 / 16;
                    double RPM = (((-shooterMotor.getVelocity()) / 28) * 60) * gearRatio;
                    double kV = kVClose;
                    double ff = kSFar + (kV * currentTPS);
                    double error = currentTPS - RPM;
                    double feedback = kPFar * error;
                    double pwr = Math.max(0, ff + feedback);
                    measuredDistance = spindexDistance.getDistance(DistanceUnit.MM);
                    isShooting = (RPM >= GOAL_MIN_CLOSE_RPM);
                    shooterMotor.setPower(pwr);
                    if (isShooting) {
                        setPathState(PathState.SHOOT_POS);
                        telemetry.addLine("SHOOTING MECHANISM IS READY");
                    }
                    break;
                }
            }
            case SHOOT_POS: {
                if(!follower.isBusy()) {
                    startShooting();
                    if(pathTimer.getElapsedTimeSeconds() >= 3) {
                        stopShooting();
                        setPathState(PathState.DRIVE_TO_END_POS);
                    }
                    break;
                }
            }
            case DRIVE_TO_END_POS: {
                if(!endPathStarted) {
                    endPositionChain = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            follower.getPose(),
                                            new Pose(endPosition.getX(), endPosition.getY())
                                    )
                            )
                            .setConstantHeadingInterpolation(endPosition.getHeading())
                            .build();
                    follower.followPath(endPositionChain, true);
                    endPathStarted = true;
                }
                if(!follower.isBusy()) {
                    setPathState(PathState.IDLE);
                }
                break;
            }
            case IDLE: {
                follower.breakFollowing();
                break;
            }
            default: {
                telemetry.addLine("Not in an expected state.");
                break;
            }
        }
    }
    public void setPathState(PathState newstate) {
        pathstate = newstate;
        telemetry.addData("Ended state in: ", pathTimer.getElapsedTimeSeconds());
        pathTimer.resetTimer();
        if(newstate == PathState.DRIVE_TO_END_POS) {
            endPathStarted = false;
        }
    }
    public void intake() {
        //INTAKE LOGIC FROM TELEOP, CONVERT (and more)
        intake.setPower(-1);
    }
    public void stopIntake() {
        intake.setPower(0);
        spindexerServo.setPosition(spindexerPos[2]);
    }
    public void startShooting() {
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
        shooterMotor.setPower(pwr);
        aimTurretFromOdometry(2);
        transfer(true);
        transferMotor.setPower(0.8);
        intake.setPower(-0.3);
        if (isShooting) {
            if (currentTimer.milliseconds() >= nextShootAdvanceTime && spinidx < 6) {
                spinidx++; spinidx %= 6; spinidx = Math.max(3, spinidx);
                nextShootAdvanceTime = currentTimer.milliseconds() + SHOOT_ADVANCE_MS_FAST;
            }
        }
        spindexerServo.setPosition(spindexerPos[spinidx]);
    }
    public void stopShooting() {
        transfer(false);
        spinidx = 0;
        shooterMotor.setPower(kSFar);
        transferMotor.setPower(0);
        intake.setPower(0);
    }

}