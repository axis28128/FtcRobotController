package org.firstinspires.ftc.teamcode.axis28128;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.PathCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Mat;

@Autonomous (name = "leave")
public class leave extends LinearOpMode {

    public PathChain leave;
    public Pose start=new Pose(56, 8,Math.toRadians(180)),
            leavePose=new Pose(20, 8, Math.toRadians(180));

    public static Follower follower;

    public void buildPaths(){
        leave=follower.pathBuilder()
                .addPath(new BezierLine(start,leavePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }


    public void runOpMode(){
        follower= Constants.createFollower(hardwareMap);
        while (opModeInInit()){
            follower.setStartingPose(start);
        }

        buildPaths();

        waitForStart();
        follower.followPath(leave);

        while (opModeIsActive()){
            follower.update();

        }
    }

}
