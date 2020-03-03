package org.firstinspires.ftc.teamcode.Auto_Routines;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name ="RR si OpenCv",group = "drive")

public class RR_OpenCv extends LinearOpMode {
    public static double DISTANCE = 30;
    private OpenCvWebcam webcam;
    private SkystoneDetector skystoneDetector;
    public static double yellow = 100;
    public static int gray = 40;
    public static String position = null;
    public ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized robot = new SampleMecanumDriveREVOptimized(hardwareMap);
        robot.led.enableLed(false);

        //initialize webcam
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class,"camera porno"));
        webcam.openCameraDevice();

        //set up skystone detector
        skystoneDetector = new SkystoneDetector();
        skystoneDetector.blackFilter = new GrayscaleFilter(0, gray);
        skystoneDetector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, yellow);

        //assign webcam to use skystone detector
        webcam.setPipeline(skystoneDetector);
        webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        //stream to dashboard
        FtcDashboard ftcDashboard = FtcDashboard.getInstance();
        ftcDashboard.startCameraStream(webcam, 30);

        //add telemetry to DS

        //add telemetry to dashboard
        Telemetry dashboardTelemetry = ftcDashboard.getTelemetry();

        waitForStart();

        if (isStopRequested()) return;
        position = detecteaza(robot);
        runtime.reset();
        if(position == "dreapta") tr_dr(robot);
        else if(position == "mijloc") tr_mij(robot);
        else tr_stg(robot);

    }

    //cod detectare skystone

    public String detecteaza(SampleMecanumDriveREVOptimized robot){
        double x_position =0 , y_position= 0;
        sleep(200);

        runtime.reset();
        while (runtime.seconds() < 0.8){
            x_position = skystoneDetector.getScreenPosition().x;
            y_position = skystoneDetector.getScreenPosition().y;
        }

        if( x_position < 330){
            return "stanga";
        }else if( x_position > 380 && x_position< 500){
            return "mijloc";
        }else return "dreapta";
    }

    public void tr_dr(SampleMecanumDriveREVOptimized robot){

    }
    public void tr_mij(SampleMecanumDriveREVOptimized robot){

    }
    public void tr_stg(SampleMecanumDriveREVOptimized robot){
        Trajectory traj0 = robot.trajectoryBuilder()
                .back(6.5)
                .strafeLeft(54)
                .build();
        Trajectory traj1 = robot.trajectoryBuilder()
                .reverse()
                .strafeRight(6)
                .reverse()
                .back(137)
                .strafeLeft(11.5)
                .build();
        Trajectory traj2 = robot.trajectoryBuilder()
                .strafeRight(2.5)
                .forward(100)
                .addMarker(()->{
                    robot.brat.setPosition(0);
                    sleep(200);
                    robot.claw.setPosition(0.5);
                    return null;
                })
                .forward(69)
                .build();
        Trajectory traj3 = robot.trajectoryBuilder()
                .strafeLeft(5.75)
                .build();
        Trajectory traj4 = robot.trajectoryBuilder()
                .strafeRight(3.5)
                .back(158)
                .strafeLeft(9.5)
                .build();
        Trajectory traj5 = robot.trajectoryBuilder()
                .strafeRight(3)
                .build();
        Trajectory traj6 = robot.trajectoryBuilder()
                .strafeRight(4)
                .back(8)
                .build();
        Trajectory traj7 = robot.trajectoryBuilder()
                .forward(37)
                .build();
        Trajectory traj8 = robot.trajectoryBuilder()
                .forward(2)
                .strafeLeft(40)
                .back(25)
                .strafeLeft(20)
                .build();

        robot.prins_fundatie_stg.setPosition(0.8);//corect
        robot.prins_fundatie_dr.setPosition(0);
        robot.brat.setPosition(0);
        robot.claw.setPosition(0.5);//in sus
        sleep(500);
        robot.followTrajectorySync(traj0);
        robot.claw.setPosition(0);
        sleep(700);
        robot.brat.setPosition(0.5);
        sleep(200);
        robot.followTrajectorySync(traj1);
        robot.brat.setPosition(0.05);
        sleep(200);
        robot.claw.setPosition(0.6);
        robot.brat.setPosition(0.5);
        sleep(600);
        robot.followTrajectorySync(traj2);
        robot.followTrajectorySync(traj3);
        robot.claw.setPosition(0);
        sleep(620);
        robot.brat.setPosition(0.5);
        robot.followTrajectorySync(traj4);
        robot.brat.setPosition(0.1);
        sleep(300);
        robot.claw.setPosition(0.5);
        sleep(200);
        robot.brat.setPosition(0.45);
        robot.followTrajectorySync(traj5);
        robot.turnSync(Math.toRadians(-95));
        robot.followTrajectorySync(traj6);
        robot.prins_fundatie_stg.setPosition(0);
        robot.prins_fundatie_dr.setPosition(0.6);
        sleep(450);
        robot.followTrajectorySync(traj7);
        robot.prins_fundatie_stg.setPosition(0.8);//corect
        robot.prins_fundatie_dr.setPosition(0);
        sleep(400);
        robot.followTrajectorySync(traj8);

    }


}
