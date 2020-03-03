package org.firstinspires.ftc.teamcode.Auto_Routines;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "auto rosu rr", group = "drive")

public class Auto_rosu_rr extends LinearOpMode {

    public double DISTANCE=30;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .strafeRight(DISTANCE*1.15)
                .build();
        Trajectory traj1 = drive.trajectoryBuilder()
                .strafeLeft(DISTANCE/7)
                .back(DISTANCE*3)
                .strafeRight(DISTANCE/2)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder()
                .strafeLeft(DISTANCE/7)
                .forward(DISTANCE/3*9.9)
                .strafeRight(DISTANCE/5.85)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder()
                .strafeLeft(DISTANCE/7)
                .back(DISTANCE*3)
                .strafeRight(DISTANCE/2.2)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder()
                .strafeLeft(DISTANCE/2.5)
                .back(DISTANCE/5)
                .build();
        Trajectory traj6 =drive.trajectoryBuilder()
                .forward(DISTANCE*1.3)
                .build();
        Trajectory traj7 =drive.trajectoryBuilder()
                .strafeRight(DISTANCE)
                .back(DISTANCE/1.5)
                .strafeRight(DISTANCE/2)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
        drive.prins_fundatie_dr.setPosition(0);
        drive.prins_fundatie_stg.setPosition(0.8);
        drive.brat.setPosition(0);
        sleep(500);
        drive.claw.setPosition(0);
        sleep(700);
        drive.brat.setPosition(0.25);
        sleep(300);
        drive.followTrajectorySync(traj1);


        drive.brat.setPosition(0);
        sleep(100);
        drive.claw.setPosition(0.4);
        sleep(300);
        drive.brat.setPosition(0.25);
        drive.followTrajectorySync(traj2);


        drive.brat.setPosition(0);
        sleep(200);
        drive.claw.setPosition(0);
        sleep(800);
        drive.brat.setPosition(0.25);
        drive.followTrajectorySync(traj3);


        drive.brat.setPosition(0);
        sleep(100);
        drive.claw.setPosition(0.4);
        sleep(300);
        drive.brat.setPosition(0.25);
        drive.turnSync(Math.toRadians(88));
        drive.followTrajectorySync(traj5);


        sleep(100);
        drive.prins_fundatie_stg.setPosition(0);
        drive.prins_fundatie_dr.setPosition(0.6);
        sleep(400);
        drive.followTrajectorySync(traj6);


        drive.prins_fundatie_dr.setPosition(0);
        drive.prins_fundatie_stg.setPosition(0.8);
        sleep(250);
        drive.followTrajectorySync(traj7);


    }
}
