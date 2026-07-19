package org.firstinspires.ftc.teamcode.axis28128;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Moving Ranking Point Auto")
public class MovingRankingPointAuto extends OpMode {
    private Follower follower;
    private PathChain moveBackChain;
    private Timer pathTimer;
    private boolean pathStarted = false;

    // Starting position
    private static final Pose START_POSE = new Pose(62, 8, Math.toRadians(0));

    // Position to move back to (moved back on X-axis)
    private static final Pose MOVE_BACK_POSITION = new Pose(45, 8, Math.toRadians(0));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

        follower.setPose(START_POSE);
        buildPath();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Position", "X: %.2f, Y: %.2f", START_POSE.getX(), START_POSE.getY());
        telemetry.addData("Move Back Position", "X: %.2f, Y: %.2f", MOVE_BACK_POSITION.getX(), MOVE_BACK_POSITION.getY());
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        if (!pathStarted) {
            follower.followPath(moveBackChain, true);
            pathStarted = true;
        }

        telemetry.addData("Current Position", "X: %.2f, Y: %.2f", follower.getPose().getX(), follower.getPose().getY());
        telemetry.addData("Robot Busy", follower.isBusy());
        telemetry.addData("Time Elapsed", "%.2f seconds", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    private void buildPath() {
        moveBackChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                START_POSE,
                                MOVE_BACK_POSITION
                        )
                )
                .setConstantHeadingInterpolation(START_POSE.getHeading())
                .build();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}
