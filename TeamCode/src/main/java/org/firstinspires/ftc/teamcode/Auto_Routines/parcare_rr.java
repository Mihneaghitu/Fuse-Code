package org.firstinspires.ftc.teamcode.Auto_Routines;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.opencv.core.Mat;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name ="parcare rr",group = "drive")

public class parcare_rr extends LinearOpMode {
    public static double DISTANCE = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized robot = new SampleMecanumDriveREVOptimized(hardwareMap);

     Trajectory trajectory = robot.trajectoryBuilder()
             .forward(DISTANCE)
             //.lineTo(new Vector2d(25,25),new ConstantInterpolator(Math.toRadians(0)))
             .build();

        waitForStart();

        robot.followTrajectorySync(trajectory);

        if (isStopRequested()) return;


    }
}
