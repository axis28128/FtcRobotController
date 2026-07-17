package org.firstinspires.ftc.teamcode.axis28128;

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
    public PathChain firstchain, secondchain, thirdchain, fourthchain, fifthchain, sixthchain, seventhchain, eighthchain, ninthchain;
    private static final Pose startingPose = new Pose(120, 121.4, Math.toRadians(216)); //VERY VERY IMPORTANT - STARTING POSE FOR THE AUTO
    private static final Pose shootPose = new Pose(86.457, 105.480, 0);
    public boolean middleTaken = false;
    public enum PathState {
        DRIVE_START_POS_SHOOT_POS,
        SHOOT_POS,
        SHOOT_PRELOAD,
        SHOOT_STOP,
        SHOOT_POS_GATE_INTAKE,
        GATE_INTAKE_SHOOT_POS,
        SHOOT_POS_MIDDLE_THREE,
        MIDDLE_THREE_SHOOT_POS;

    }
    public PathState pathstate;
    private Timer pathTimer, opModeTimer;

    // Add fields
    public static double BALL_DETECT_THRESHOLD = 65; // between ball (~40) and gap (~100)
    public static int BALL_DETECT_CONSECUTIVE = 7;    // readings needed to confirm
    private int FarReadingStreak = 0;
    public static double SHOOT_ADVANCE_MS_FAST = 350, SHOOT_ADVANCE_MS_SORTING = 1000; // tune this — time between each ball feed
    private double nextShootAdvanceTime = 0;
    public static double SHOOTER_POS_FAR = 0.7, SHOOTER_POS_CLOSE = 0.3;
    public boolean sorting = false, globalSorting = false;
    private Follower follower;

    public TelemetryManager telemetryM;
    //EVERYTHING THAT FOR SHOOTER RPM LOGIC
    public static double GOAL_RPM_FAR = 3200, GOAL_RPM_CLOSE = 2750, currentTPS = GOAL_RPM_CLOSE;
    public static double GOAL_MIN_CLOSE_RPM = 2600, GOAL_MIN_FAR_RPM = 3100;
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

    public double[] spindexerPos = {0.29, 0.54, 0.78, 0.6, 0.36, 0.12};
    public double measuredDistance = 0;
    public int spinidx = 0, shotBalls = 0, intakedBalls = 3;
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
        if(30 - opModeTimer.getElapsedTimeSeconds() <= 5 && !follower.isBusy()) {
            PathChain lastChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    follower.getPose(),
                                    new Pose(80, 100)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
            follower.followPath(lastChain);
            turretMotor.setTargetPosition(0);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(0.3);
        }
        else statePathUpdate();
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
        firstchain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.563, 121.422),
                                new Pose(82.404, 83.630)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(0))
                .build();
        secondchain = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(82.404, 83.630),
                                new Pose(85.494, 56.421),
                                new Pose(115.077, 57.882)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setVelocityConstraint(0.25)
                .build();
        thirdchain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(115.077, 57.882),
                                new Pose(84.300, 105.531)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        fourthchain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(82.300, 83.531),
                                new Pose(131, 56)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
        fifthchain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(131, 56),
                                new Pose(84.542, 103.450)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
        sixthchain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(82.542, 83.450),
                                new Pose(129.574, 58.362)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                .build();
        seventhchain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(129.574, 58.362),
                                new Pose(82.275, 83.461)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                .build();
        eighthchain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(82.275, 83.461),
                                new Pose(114.955, 80.852)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        ninthchain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(114.955, 80.852),
                                new Pose(84.177, 105.331)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public void statePathUpdate() {
        switch(pathstate) {
            case DRIVE_START_POS_SHOOT_POS: {
                follower.followPath(firstchain, true);
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
                        setPathState(PathState.SHOOT_STOP);
                    }
                    break;
                }
            }
            case SHOOT_STOP: {
                if(!follower.isBusy()) {
                    stopShooting();
                    intakedBalls = 0;
                    if(middleTaken) setPathState(PathState.SHOOT_POS_GATE_INTAKE);
                    else setPathState(PathState.SHOOT_POS_MIDDLE_THREE);
                    intake();
                    break;
                }
            }
            case SHOOT_POS_MIDDLE_THREE: {
                if(!follower.isBusy()) {
                    intake();
                    follower.followPath(secondchain, true);
                    setPathState(PathState.MIDDLE_THREE_SHOOT_POS);
                    middleTaken = true;
                }
                if(intakedBalls >= 3) stopIntake();
                break;
            }
            case SHOOT_POS_GATE_INTAKE: {
                if(!follower.isBusy()) {
                    intake();
                    follower.followPath(fourthchain, true);
                    middleTaken = true;
                    setPathState(PathState.GATE_INTAKE_SHOOT_POS);
                }
                break;
            }
            case MIDDLE_THREE_SHOOT_POS: {
                if(!follower.isBusy()) {
                    follower.followPath(thirdchain, true);
                    stopIntake();
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            }
            case GATE_INTAKE_SHOOT_POS: {
                if(!follower.isBusy()) {
                    follower.followPath(fifthchain, true);
                    stopIntake();
                    setPathState(PathState.SHOOT_PRELOAD);
                }
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
    }
    public void intake() {
        //INTAKE LOGIC FROM TELEOP, CONVERT (and more)
        intake.setPower(-1);
        boolean readingIsFar = measuredDistance <= BALL_DETECT_THRESHOLD;
        if (readingIsFar) {
            FarReadingStreak++;
        } else {
            FarReadingStreak = 0;
        }
        boolean ballNearNow = FarReadingStreak >= BALL_DETECT_CONSECUTIVE;
        if (ballNearNow && !ballWasDetected && spinidx < 2) {
            spinidx++;
            intakedBalls++;
        }
        else if(ballNearNow && !ballWasDetected) intakedBalls++;
        ballWasDetected = ballNearNow;
        if (spinidx > 6) spinidx = 0;
        if (spinidx < 0) spinidx = 0;
        spindexerServo.setPosition(spindexerPos[spinidx]);
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
            if (currentTimer.milliseconds() >= nextShootAdvanceTime && spinidx < 5) {
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