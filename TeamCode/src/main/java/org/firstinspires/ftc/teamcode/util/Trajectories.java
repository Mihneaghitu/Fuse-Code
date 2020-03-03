package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

public class Trajectories {
    public void traj_dr(SampleMecanumDriveREVOptimized robot){

        Trajectory traj1dr = robot.trajectoryBuilder()
                .reverse()
                .addMarker(()->{
                    robot.brat.setPosition(0.35);
                    return null;
                })
                .splineTo(new Pose2d(-40,-5,0))
                .build();
        Trajectory traj2dr = robot.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-47,20,0))
                .reverse()
                .strafeLeft(15)
                .build();
        Trajectory traj3dr = robot.trajectoryBuilder()
                .splineTo(new Pose2d(40,-5,0))
                .addMarker(()->{
                    robot.brat.setPosition(0.6);
                    return null;
                })
                .splineTo(new Pose2d())
                .build();
    }
}
